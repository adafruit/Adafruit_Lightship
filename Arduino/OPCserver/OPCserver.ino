// Wireless Open Pixel Control widget...kindasorta a little bit like
// Fadecandy, but self-contained with wireless and using a single long
// strand of DotStar LEDs instead of 8-way NeoPixels.  This is the OPC
// 'server' side -- the OPC 'client' application, which actually drives
// the animation, runs on a regular computer running the Processing
// language (www.processing.org).
// Requires either:
//   Adafruit Feather M0 WiFi: https://www.adafruit.com/products/3010
// OR:
//   Arduino Zero:             https://www.adafruit.com/products/2843
//   and WiFi Shield 101:      https://www.adafruit.com/products/2891
// Plus a length of DotStar LEDs (strip, matrix, etc.) and a power source.

// Enable this #define if using Adafruit ATWINC (e.g. Feather M0 WiFi).
// If using Arduino Zero + WiFi Shield 101, comment it out:
#define ADAFRUIT_ATWINC

//#define Serial SerialUSB // Enable if using Arduino Zero 'Native USB' port

// Scroll down to 'CONFIG & GLOBALS' section for further settings.

#define IP_STATIC  0
#define IP_DYNAMIC 1
#define IP_BONJOUR 2 // DO NOT USE - NOT 100% RELIABLE YET
#include <WiFi101.h>
#include <SPI.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"
#include "wiring_private.h" // pinPeripheral() function

// CONFIG & GLOBALS --------------------------------------------------------

#define IP_TYPE IP_STATIC // IP_STATIC | IP_DYNAMIC | IP_BONJOUR
// Bonjour support isn't stable yet, use IP_STATIC or IP_DYNAMIC for now

char       ssid[] = "NETWORK_NAME",   // WiFi credentials
           pass[] = "NETWORK_PASSWORD";
#if (IP_TYPE == IP_STATIC)
IPAddress  ipaddr(192, 168, 0, 60);   // Static IP address, if so configured
#endif
#define    INPORT 7890                // Incoming TCP port to listen on
WiFiServer server(INPORT);

// Declare second SPI peripheral 'SPI1':
SPIClass SPI1(      // 11/12/13 classic UNO-style SPI
  &sercom1,         // -> Sercom peripheral
  12,               // MISO pin
  13,               // SCK pin
  11,               // MOSI pin
  SPI_PAD_0_SCK_1,  // TX pad (for MOSI, SCK)
  SERCOM_RX_PAD_3); // RX pad (for MISO)

#define MAX_LEDS        512 // Upper limit; OK to receive data for fewer
#define SPI_BUFFER_SIZE (4 + MAX_LEDS * 4 + ((MAX_LEDS / 2) + 7) / 8)
// SPI buffer includes space for DotStar 32-bit '0' header, 32 bits per LED,
// and footer of 1 bit per 2 LEDs (rounded to next byte boundary, for SPI).
// For 512 pixels, that's 2084 bytes per SPI buffer (x2 = 4168 bytes total).

// Two equal-size SPI buffers are allocated; one's being filled with new
// data as the other's being issued via DMA.
uint8_t spiBuffer[2][SPI_BUFFER_SIZE];
uint8_t spiBufferBeingFilled = 0;    // Index of currently-calculating buf
volatile bool spiReady       = true; // True when SPI DMA ready for new data

// Data for most-recently-received OPC color payload, payload before that,
// and new in-progress payload currently arriving.  Also, a 'sink' buffer
// for quickly discarding data.
uint8_t rgbBuf[4][MAX_LEDS * 3]; // 512 LEDs = 6144 bytes.

// These tables (computed at runtime) are used for gamma correction and
// dithering.  RAM used = 256*9+MAX_LEDS*3 bytes.  512 LEDs = 3840 bytes.
uint8_t loR[256], hiR[256], fracR[256], errR[MAX_LEDS],
        loG[256], hiG[256], fracG[256], errG[MAX_LEDS],
        loB[256], hiB[256], fracB[256], errB[MAX_LEDS];

// Order of color bytes as issued to the DotStar LEDs.  Current DotStars use
// BGR order (blue=first, green=second, red=last); pre-2015 DotStars use GBR
// order.  THESE RELATE ONLY TO THE LED STRIP; OPC data is always RGB order.
#define DOTSTAR_BLUEBYTE  0
#define DOTSTAR_GREENBYTE 1
#define DOTSTAR_REDBYTE   2

Adafruit_ZeroDMA myDMA; // For DMA transfers
DmacDescriptor  *desc;  // DMA descriptor address

// UTILITY FUNCTIONS -------------------------------------------------------

// Called each time a DMA transfer finishes
void dma_callback(Adafruit_ZeroDMA *dma) {
  spiReady = true; // OK to issue next SPI DMA payload now!
}

// Compute gamma/dither tables for one color component.  Pass gamma and max
// brightness level (e.g. 2.7, 255) followed by pointers to 'lo', 'hi' and
// 'frac' tables to fill.  Typically will call this 3 times (R, G, B).
void fillGamma(float g, uint8_t m, uint8_t *lo, uint8_t *hi, uint8_t *frac) {
  uint16_t i, j, n;
  for(i=0; i<256; i++) {
    // Calc 16-bit gamma-corrected level
    n = (uint16_t)(pow((double)i / 255.0, g) * (double)m * 256.0 + 0.5);
    lo[i]   = n >> 8;   // Store as 8-bit brightness level
    frac[i] = n & 0xFF; // and 'dither up' probability (error term)
  }
  // Second pass, calc 'hi' level for each (based on 'lo' value)
  for(i=0; i<256; i++) {
    n = lo[i];
    for(j=i; (j<256) && (lo[j] <= n); j++);
    hi[i] = lo[j];
  }
}

// -------------------------------------------------------------------------

// This function interpolates between two RGB input buffers, gamma- and
// color-corrects the interpolated result with 16-bit dithering and issues
// the resulting data to the GPIO port.
void magic(
 uint8_t *rgbIn1,    // First RGB input buffer being interpolated
 uint8_t *rgbIn2,    // Second RGB input buffer being interpolated
 uint8_t  w2,        // Weighting (0-255) of second buffer in interpolation
 uint8_t *fillBuf,   // SPI data buffer being filled (DotStar-native order)
 uint16_t numLEDs) { // Number of LEDs in buffer
  uint8_t   mix;
  uint16_t  weight1, weight2, pixelNum, e;
  uint8_t  *fillPtr = fillBuf + 5; // Skip 4-byte header + 1 byte pixel marker

  weight2 = (uint16_t)w2 + 1; // 1-256
  weight1 = 257 - weight2;    // 1-256

  for(pixelNum = 0; pixelNum < numLEDs; pixelNum++, fillPtr += 4) {
    // Interpolate red from rgbIn1 and rgbIn2 based on weightings
    mix = (*rgbIn1++ * weight1 + *rgbIn2++ * weight2) >> 8;
    // fracR is the fractional portion (0-255) of the 16-bit gamma-
    // corrected value for a given red brightness...essentially it's
    // how far 'off' a given 8-bit brightness value is from its ideal.
    // This error is carried forward to the next frame in the errR
    // buffer...added to the fracR value for the current pixel...
    e = fracR[mix] + errR[pixelNum];
    // ...if this accumulated value exceeds 255, the resulting red
    // value is bumped up to the next brightness level and 256 is
    // subtracted from the error term before storing back in errR.
    // Diffusion dithering is the result.
    fillPtr[DOTSTAR_REDBYTE] = (e < 256) ? loR[mix] : hiR[mix];
    // If e exceeds 256, it *should* be reduced by 256 at this point...
    // but rather than subtract, we just rely on truncation in the 8-bit
    // store operation below to do this implicitly. (e & 0xFF)
    errR[pixelNum] = e;

    // Repeat same operations for green...
    mix = (*rgbIn1++ * weight1 + *rgbIn2++ * weight2) >> 8;
    e   = fracG[mix] + errG[pixelNum];
    fillPtr[DOTSTAR_GREENBYTE] = (e < 256) ? loG[mix] : hiG[mix];
    errG[pixelNum] = e;

    // ...and blue...
    mix = (*rgbIn1++ * weight1 + *rgbIn2++ * weight2) >> 8;
    e   = fracB[mix] + errB[pixelNum];
    fillPtr[DOTSTAR_BLUEBYTE] = (e < 256) ? loB[mix] : hiB[mix];
    errB[pixelNum] = e;
  }

  while(!spiReady); // Wait for prior SPI DMA transfer to complete

  // Modify the DMA descriptor using the newly-filled buffer as source...
  myDMA.changeDescriptor(desc, // DMA descriptor address
    fillBuf);                  // New src only; dst & count don't change

  spiReady = false;
  myDMA.startJob();
}

// SETUP (one-time init) ---------------------------------------------------

void setup() {
  Serial.begin(115200);
  // while(!Serial); // Waits for serial monitor to be opened
  Serial.println("OPC WiFi Server");

  SPI.begin(); // For WiFi interface

  // Connect to WiFi network
#ifdef ADAFRUIT_ATWINC
  WiFi.setPins(8, 7, 4, 2); // Pins for Adafruit ATWINC1500 Feather
#endif
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print("..");
#if (IP_TYPE == IP_STATIC)
  WiFi.config(ipaddr);
#endif
  WiFi.begin(ssid, pass);
  // Do some other init while WiFi starts up...

  // Initialize SPI buffers.  Everything's set to 0xFF initially to cover
  // the per-pixel 0xFF marker and the end-of-data '1' bits, then the first
  // 4 bytes of each buffer are set to 0x00 as start-of-data marker.
  memset(spiBuffer, 0xFF, sizeof(spiBuffer));
  for(uint8_t b=0; b<2; b++) {
    for(uint8_t i=0; i<4; i++) spiBuffer[b][i] = 0x00;
  }

  fillGamma(2.7, 255, loR, hiR, fracR); // Initialize gamma tables to
  fillGamma(2.7, 255, loG, hiG, fracG); // default values (OPC data may
  fillGamma(2.7, 255, loB, hiB, fracB); // override this later).
  // err buffers don't need init, they'll naturally reach equilibrium

  memset(rgbBuf, 0, sizeof(rgbBuf)); // Clear receive buffers

  SPI1.begin();                  // Init second SPI bus
  pinPeripheral(11, PIO_SERCOM); // Enable SERCOM MOSI on this pin
  pinPeripheral(13, PIO_SERCOM); // Ditto, SERCOM SCK
  SPI1.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
  // Long DotStar stips may require reducing the SPI clock; if you see
  // glitching, try setting to 8 MHz above.

  // Configure DMA for SERCOM1 (our 'SPI1' port on 11/12/13)
  myDMA.setTrigger(SERCOM1_DMAC_ID_TX);
  myDMA.setAction(DMA_TRIGGER_ACTON_BEAT);
  myDMA.allocate();
  desc = myDMA.addDescriptor(
    NULL,                             // Source address (not set yet)
    (void *)(&SERCOM1->SPI.DATA.reg), // Dest address
    SPI_BUFFER_SIZE,                  // Data count
    DMA_BEAT_SIZE_BYTE,               // Bytes/halfwords/words
    true,                             // Increment source address
    false);                           // Don't increment dest
  myDMA.setCallback(dma_callback);

  // Turn off LEDs
  magic(rgbBuf[0], rgbBuf[0], 0, spiBuffer[spiBufferBeingFilled], MAX_LEDS);
  spiBufferBeingFilled = 1 - spiBufferBeingFilled;

  while(WiFi.status() != WL_CONNECTED) {
    Serial.write('.');
    delay(500);
  }
  Serial.println("OK!");

  // Print the IP address & port
#if (IP_TYPE == IP_BONJOUR)
  if(!mdns.begin(mdns_name)) {
    Serial.println("Failed to start MDNS responder!");
    for(;;);
  }

  Serial.print("Server listening at ");
  Serial.print(mdns_name);
  Serial.print(".local");
#else // IP_STATIC or IP_DYNAMIC
  Serial.print("Server listening at ");
  Serial.print((IPAddress)WiFi.localIP());
#endif

  // Start the server listening for incoming client connections
  server.begin();
  Serial.print(" port ");
  Serial.println(INPORT);
}

// OPC-HANDLING LOOP -------------------------------------------------------

// Getting decent interpolation and dithering REQUIRES frequent calls to
// the magic() function.  This means we can't sit in a tight loop waiting
// for an OPC packet to arrive.  Instead, loop() interneaves between the
// two tasks: handling a single magic() call, then receiving a finite amount
// of data from the OPC client.  A crude state machine cycles between three
// states as needed: MODE_HEADER is waiting for a 4-byte OPC header to
// arrive, MODE_DATA is receiving a pixel data 'payload' (typically only a
// smaller chunk of this data is read on each pass through loop()), and
// MODE_DISCARD which flushes unneeded data (either unsupported command
// packets or excess pixel data beyond what magic() supports) from the
// client (this also operates on smaller chunks per pass).

#define MODE_DATA    0
#define MODE_HEADER  1
#define MODE_DISCARD 2
uint8_t mode = MODE_HEADER;

// bytesToRead is the number of bytes remaining in current mode, while
// bytesRead is the number read so far (used as an index into a destination
// buffer).  bytesToDiscard is the number remaining when in MODE_DISCARD.
int16_t bytesToRead = 0, bytesRead = 0, bytesToDiscard = 0,
        numLEDs = MAX_LEDS, nextNumLEDs = MAX_LEDS;

// There are three LED data buffers: one currently being read (not
// displayed yet), the one most recently fully read (a complete RGB
// pixel payload, currently being interpolated) and the one before that
// (also a full RGB payload, also part of the interpolation).
// These indices keep track of each, and all are incremented whenever a
// full RGB payload is received (resetting back to 0 as necessary).
uint8_t bufPrior = 0, bufMostRecentlyRead = 1, bufBeingRead = 2;
// There's also a fourth RGB buffer used as a 'bit bucket' when discarding
// data quickly.  It's always index #3 and doesn't need a variable.

// For interpolation: lastFrameTime is the absolute time (in microseconds)
// when the most recent OPC pixel data packet was fully read.
// timeBetweenFrames is the interval (also in cycles) between lastFrameTime
// and the frame before that.  updates and priorSeconds are used for the
// updates-per-second estimate.
uint32_t lastFrameTime = 0, timeBetweenFrames = 0,
         updates = 0, priorSeconds  = 0;

void loop() {
#if (IP_TYPE == IP_BONJOUR)
  mdns.update();
#endif
  WiFiClient client = server.available();

  if(client) {
    uint32_t t, timeSinceFrameStart, seconds;
    int16_t  a, bytesPending, dataSize;
    uint8_t  w;
    Serial.println("new client");
    while(client.connected()) {

      // DITHER-AND-RECEIVE LOOP STARTS HERE -------------------------------

      // Interpolation weight (0-255) is the ratio of the time since last
      // frame arrived to the prior two frames' interval.
      t                   = micros();          // Current time
      timeSinceFrameStart = t - lastFrameTime; // Elapsed since data recv'd
      w                   = (timeSinceFrameStart >= timeBetweenFrames) ? 255 :
                            (255L * timeSinceFrameStart / timeBetweenFrames);

      magic(rgbBuf[bufPrior], rgbBuf[bufMostRecentlyRead], w,
        spiBuffer[spiBufferBeingFilled], numLEDs);
      spiBufferBeingFilled = 1 - spiBufferBeingFilled;
      updates++;

      // Show approximate updates-per-second
      if((seconds = (t / 1000000)) != priorSeconds) { // 1 sec elapsed?
        Serial.print(updates);
        Serial.println(" updates/sec");
        priorSeconds = seconds;
        updates      = 0; // Reset counter
#if (IP_TYPE == IP_BONJOUR)
        mdns.update();
#endif
      }

      // Process up to 1/2 of pending data on stream.  Rather than waiting
      // for full packets to arrive, this interleaves Stream I/O with LED
      // dithering so the latter doesn't get too 'stuttery.'  It DOES
      // however limit the potential throughput; 256 LEDs seems fine at
      // 60 FPS, but with 512 you may need to limit it to 30 FPS.
      if(bytesPending = client.available()) {  // Any incoming data?
        bytesPending = (bytesPending + 1) / 2; // Handle a fraction of it
        do {
          if(mode == MODE_DATA) { // Receiving pixel data, most likely case
            // Read size mustn't exceed remaining pixel payload size or pixel
            // buffer size. This avoids some ugly cases like the next pixel
            //header appearing mid-buffer, which would require nasty memmoves
            // and stuff.  We'll read some now and pick up the rest on the
            // next pass through.
            if(bytesPending>bytesToRead)       bytesPending=bytesToRead;
            if(bytesPending>sizeof(rgbBuf[0])) bytesPending=sizeof(rgbBuf[0]);
            if((a = client.read(&rgbBuf[bufBeingRead][bytesRead],
             bytesPending)) > 0) {
              bytesPending -= a;
              bytesToRead  -= a;
              if(bytesToRead <= 0) { // End of pixel payload?
                // END OF PIXEL DATA.  Record arrival time and interval since
                // last frame, advance buffer indices, switch to HEADER mode
                // if end of packet, else to DISCARD mode for any remainders.
                t                   = micros();
                timeBetweenFrames   = t - lastFrameTime;
                lastFrameTime       = t;
                bufPrior            = bufMostRecentlyRead; // Cycle buffers
                bufMostRecentlyRead = bufBeingRead;
                bufBeingRead        = (bufBeingRead + 1) % 3;
                numLEDs             = nextNumLEDs;
                bytesRead           = 0; // Reset index
                mode = bytesToDiscard ? MODE_DISCARD : MODE_HEADER;
              } else {
                bytesRead += a; // Advance index & keep reading
              }
            } // else no data received
          } else if(mode == MODE_HEADER) {         // Receiving header data
            if(bytesPending > 4) bytesPending = 4; // Limit to header size
            if((a = client.read(&rgbBuf[bufBeingRead][bytesRead],
             bytesPending)) > 0) {
              bytesRead    += a;
              bytesPending -= a;
              if(bytesPending <= 0) { // Full header received, parse it!
                bytesRead = 0;        // Reset read buffer index
                dataSize  = (rgbBuf[bufBeingRead][2] << 8) |
                             rgbBuf[bufBeingRead][3];
                if(dataSize > 0) {           // Payload size > 0?
                  mode = MODE_DISCARD;       // Assume DISCARD until validated,
                  bytesToDiscard = dataSize; // may override below
                  if(rgbBuf[bufBeingRead][0] <= 1) {   // Valid channel?
                    if(rgbBuf[bufBeingRead][1] == 0) { // Pixel data command?
                      // Valid!  Switch to DATA mode, set up counters...
                      mode = MODE_DATA;
                      if(dataSize <= sizeof(rgbBuf[0])) {    // <= MAX_LEDS
                        bytesToRead     = dataSize;          // Read all,
                        bytesToDiscard  = 0;                 // no discard
                      } else {                               // > MAX_LEDS
                        bytesToRead     = sizeof(rgbBuf[0]); // Read MAX_LEDS,
                        bytesToDiscard -= sizeof(rgbBuf[0]); // discard rest
                      }
                      nextNumLEDs = bytesToRead / 3; // Pixel count when done
                    } // endif valid command
                  } // endif valid channel
                } // else 0-byte payload; remain in HEADER mode
              } // else full header not yet received; remain in HEADER mode
            } // else no data received
          } else { // MODE_DISCARD
            // Read size mustn't exceed discard size or pixel buffer size
            if(bytesPending>bytesToDiscard)    bytesPending=bytesToDiscard;
            if(bytesPending>sizeof(rgbBuf[3])) bytesPending=sizeof(rgbBuf[3]);
            if((a = client.read(&rgbBuf[3][0], bytesPending)) > 0) { // Poof!
              bytesPending -= a;
              if(bytesPending <= 0) { // End of DISCARD mode,
                mode = MODE_HEADER;   // switch back to HEADER mode
              }
            }
          } // end MODE_DISCARD
        } while(bytesPending > 0);
      } // end if client.available()

      // DITHER-AND-RECEIVE LOOP ENDS HERE ---------------------------------

    } // end while client connected
    client.stop();
    Serial.println("client disonnected");
    // Clear buffers and turn off LEDs:
    memset(rgbBuf, 0, sizeof(rgbBuf));
    numLEDs = nextNumLEDs = 512;
    magic(rgbBuf[0], rgbBuf[0], 0, spiBuffer[spiBufferBeingFilled], numLEDs);
    spiBufferBeingFilled = 1 - spiBufferBeingFilled;
  }
}
