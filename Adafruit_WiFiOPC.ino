// Wireless Open Pixel Control widget...kindasorta a little bit like
// Fadecandy, but self-contained with wireless and using a single long
// strand of DotStar LEDs instead of 8-way NeoPixels.  This is the OPC
// 'server' side -- the OPC 'client' application, which actually drives
// the animation, runs on a regular computer running the Processing
// language (www.processing.org).
// Requires either:
//   Arduino Zero:             https://www.adafruit.com/products/2843
//   and WiFi Shield 101:      https://www.adafruit.com/products/2891
// OR:
//   Adafruit Feather M0 WiFi: https://www.adafruit.com/products/3010
// Plus a length of DotStar LEDs (strip, matrix, etc.) and a power source.

#include <SPI.h>
#include <WiFi101.h>
#include <Adafruit_ZeroDMA.h>
#include <Adafruit_ASFcore.h>
#include "utility/dmac.h"
#include "utility/dma.h"

//#define Serial SerialUSB // Enable if using Arduino Zero 'Native USB' port

// CONFIG & GLOBALS --------------------------------------------------------

char      *ssid = "NETWORK_NAME",  // WiFi credentials
          *pass = "NETWORK_PASSWORD";
#define    INPORT  7890            // TCP port to listen on
WiFiServer server(INPORT);

// Declare second SPI peripheral 'SPI1':
SPIClass SPI1(      // 11/12/13 classic UNO-style SPI
  &sercom1,         // -> Sercom peripheral
  34,               // MISO pin (also digital pin 12)
  37,               // SCK pin  (also digital pin 13)
  35,               // MOSI pin (also digital pin 11)
  SPI_PAD_0_SCK_1,  // TX pad (for MOSI, SCK)
  SERCOM_RX_PAD_3); // RX pad (for MISO)

#define NUM_LEDS        512 // Upper limit; OK to receive data for fewer
#define SPI_BUFFER_SIZE (4 + NUM_LEDS * 4 + ((NUM_LEDS / 2) + 7) / 8)
// SPI buffer includes space for DotStar 32-bit '0' header, 32 bits per LED,
// and footer of 1 bit per 2 LEDs (rounded to next byte boundary, for SPI).
// For 512 pixels, that's 2084 bytes per SPI buffer (x2 = 4168 bytes total).

// Two equal-size SPI buffers are allocated; one's being filled with new
// data as the other's being issued via DMA.
uint8_t spiBuffer[2][SPI_BUFFER_SIZE];
uint8_t spiBufferBeingFilled = 0;    // Index of currently-calculating buf
volatile bool spiReady       = true; // True when SPI DMA ready for new data

// Data for most-recently-received OPC color payload, payload before that,
// and new in-progress payload currently arriving.  512 LEDs = 4608 bytes.
uint8_t rgbBuf[3][NUM_LEDS * 3];

// These tables (computed at runtime) are used for gamma correction and
// dithering.  RAM used = 256*9+NUM_LEDS*3 bytes.  512 LEDs = 3840 bytes.
uint8_t loR[256], hiR[256], fracR[256], errR[NUM_LEDS],
        loG[256], hiG[256], fracG[256], errG[NUM_LEDS],
        loB[256], hiB[256], fracB[256], errB[NUM_LEDS];

// Order of color bytes as issued to the DotStar LEDs.  Current DotStars use
// BRG order (blue=first, red=second, green=last); pre-2015 DotStars use GBR
// order.  THESE RELATE ONLY TO THE LED STRIP; OPC data is always RGB order.
#define DOTSTAR_GREENBYTE 0
#define DOTSTAR_BLUEBYTE  1
#define DOTSTAR_REDBYTE   2

Adafruit_ZeroDMA myDMA; // For DMA transfers

// UTILITY FUNCTIONS -------------------------------------------------------

// Called each time a DMA transfer finishes
void dma_callback(struct dma_resource* const resource) {
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

// SETUP (one-time init) ---------------------------------------------------

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("OPC WiFi Server");

  SPI.begin(); // For WiFi Shield 101

  // Connect to WiFi network
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print("..");
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

  memset(rgbBuf, 0, sizeof(rgbBuf));    // Clear receive buffers

  SPI1.begin();                         // Init second SPI bus

  // Configure DMA for SERCOM1 (our 'SPI1' port on 11/12/13)
  myDMA.configure_peripheraltrigger(SERCOM1_DMAC_ID_TX);
  myDMA.configure_triggeraction(DMA_TRIGGER_ACTON_BEAT);
  myDMA.allocate();
  myDMA.add_descriptor();
  myDMA.register_callback(dma_callback);
  myDMA.enable_callback();

  // Turn off LEDs
  magic(rgbBuf[0], rgbBuf[0], 0, spiBuffer[spiBufferBeingFilled], NUM_LEDS);
  spiBufferBeingFilled = 1 - spiBufferBeingFilled;

  while(WiFi.status() != WL_CONNECTED) {
    Serial.write('.');
    delay(500);
  }
  Serial.println("OK!");

  // Print the IP address
  Serial.println((IPAddress)WiFi.localIP());

  // Start the server listening for incoming client connections
  server.begin();
  Serial.print("Server listening on port ");
  Serial.println(INPORT);
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
  uint16_t  weight1, weight2, byteNum, pixelNum, e;
  uint8_t  *fillPtr = fillBuf + 5; // Skip 4-byte header + 1 byte pixel marker

  weight2 = (uint16_t)w2 + 1; // 1-256
  weight1 = 257 - weight2;    // 1-256

  for(byteNum = pixelNum = 0; pixelNum < numLEDs; pixelNum++) {
    // Interpolate red from rgbIn1 and rgbIn2 based on weightings
    mix = (rgbIn1[byteNum] * weight1 + rgbIn2[byteNum] * weight2) >> 8;
    // fracR is the fractional portion (0-255) of the 16-bit gamma-
    // corrected value for a given red brightness...essentially it's
    // how far 'off' a given 8-bit brightness value is from its ideal.
    // This error is carried forward to the next pixel (and on down
    // the line) in the errR buffer...added to the fracR value for the
    // current pixel...
    e = fracR[mix] + errR[pixelNum];
    // ...if this accumulated value exceeds 255, the resulting red
    // value is bumped up to the next brightness level and 256 is
    // subtracted from the error term before storing back in errR.
    // Diffusion dithering is the result.
    if(e < 256) {                          // Error is below threshold,
      fillPtr[DOTSTAR_REDBYTE] = loR[mix]; //  use dimmer color
    } else {                               // Error at or above threshold,
      fillPtr[DOTSTAR_REDBYTE] = hiR[mix]; //  use brighter color,
      e                       -= 256;      //  reduce error value
    }
    errR[pixelNum] = e; // Store modified error term back in buffer
    byteNum++;

    // Repeat same operations for green...
    mix = (rgbIn1[byteNum] * weight1 + rgbIn2[byteNum] * weight2) >> 8;
    if((e = (fracG[mix] + errG[pixelNum])) < 256) {
      fillPtr[DOTSTAR_GREENBYTE] = loG[mix];
    } else {
      fillPtr[DOTSTAR_GREENBYTE] = hiG[mix];
      e                         -= 256;
    }
    errG[pixelNum] = e;
    byteNum++;

    // ...and blue...
    mix = (rgbIn1[byteNum] * weight1 + rgbIn2[byteNum] * weight2) >> 8;
    if((e = (fracB[mix] + errB[pixelNum])) < 256) {
      fillPtr[DOTSTAR_BLUEBYTE] = loB[mix];
    } else {
      fillPtr[DOTSTAR_BLUEBYTE] = hiB[mix];
      e                        -= 256;
    }
    errB[pixelNum] = e;
    byteNum++;

    fillPtr += 4; // Advance 4 bytes in dest buffer (0xFF + R + G + B)
  }

  while(!spiReady);      // Wait for prior SPI DMA transfer to complete
  SPI1.endTransaction(); // End prior transaction, start anew...

  // Set up DMA transfer using the newly-filled buffer as source...
  myDMA.setup_transfer_descriptor(
    fillBuf,                          // Source address
    (void *)(&SERCOM1->SPI.DATA.reg), // Dest address
    SPI_BUFFER_SIZE,                  // Data count
    DMA_BEAT_SIZE_BYTE,               // Bytes/halfwords/words
    true,                             // Increment source address
    false);                           // Don't increment dest

  // Long DotStar stips require reducing the SPI clock; 8 MHz seems
  // OK for 256 pixels, may need to go slower for 512.
  SPI1.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  spiReady = false;
  myDMA.start_transfer_job();
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

#define MODE_HEADER  0
#define MODE_DATA    1
#define MODE_DISCARD 2
uint8_t mode = MODE_HEADER;
uint8_t headerBuf[4];

// minBytesToProcess is a least amount of data that must be waiting from the
// client before it's read...any less than this and it's ignored, we make
// another pass through magic().  This is because magic() can take a couple
// milliseconds to run and we'd quickly fall behind if it was called every
// time a single byte arrived.  Conversely, maxBytesToProcess sets an upper
// limit for each pass, in order to keep the interpolation and dithering
// going (reading full packets would stall that).
uint16_t minBytesToProcess = 4, maxBytesToProcess = 4;
// Both are initially set to 4 because that's the size of an OPC packet
// header, and MODE_HEADER is the startup state.  The values change as we
// switch between different states.

// bytesToRead is the number of bytes remaining in MODE_DATA, while
// bytesRead is the number read so far (used as an index into a destination
// buffer).  bytesToDiscard is the number remaining when in MODE_DISCARD.
int16_t bytesToRead, bytesRead, bytesToDiscard,
        numLEDs = NUM_LEDS, nextNumLEDs = NUM_LEDS;

// There are three LED data buffers: one currently being read (not
// displayed yet), the one most recently fully read (a complete RGB
// pixel payload, currently being interpolated) and the one before that
// (also a full RGB payload, also part of the interpolation).
// These indices keep track of each, and all are incremented whenever a
// full RGB payload is received (resetting back to 0 as necessary).
uint8_t bufPrior = 0, bufMostRecentlyRead = 1, bufBeingRead = 2;

// For interpolation: lastFrameTime is the absolute time (in microseconds)
// when the most recent OPC pixel data packet was fully read.
// timeBetweenFrames is the interval (also in cycles) between lastFrameTime
// and the frame before that.  pt is a prior time (in seconds) used for
// the updates-per-second estimate.
uint32_t lastFrameTime = 0, timeBetweenFrames = 0, pt = 0, updates = 0;

void loop() {
  WiFiClient client = server.available();
  if(client) {
    Serial.println("new client");
    while(client.connected()) {

      // Interpolation weight (0-255) is the ratio of the time since last
      // frame arrived to the prior two frames' interval.
      uint32_t timeSinceFrameStart = micros() - lastFrameTime;
      uint8_t  w = (timeSinceFrameStart >= timeBetweenFrames) ? 255 :
                   (255L * timeSinceFrameStart / timeBetweenFrames);

      magic(rgbBuf[bufPrior], rgbBuf[bufMostRecentlyRead], w,
        spiBuffer[spiBufferBeingFilled], numLEDs);
      spiBufferBeingFilled = 1 - spiBufferBeingFilled;
      updates++;

      // Show approximage updates-per-second
      uint32_t t = millis() / 1000; // Second counter
      if(t != pt) {
        Serial.print(updates);
        Serial.println(" updates/sec");
        pt      = t;
        updates = 0;
      }

      int16_t a = client.available(); // How much data awaits?
      if(a >= minBytesToProcess) {    // Enough to bother with?
        if(a > maxBytesToProcess) a = maxBytesToProcess; // Yes, but limit
        if(mode == MODE_HEADER) {     // In header-reading mode...
          client.read(headerBuf, sizeof(headerBuf));
          uint16_t dataSize = (headerBuf[2] << 8) | headerBuf[3]; // Payload
          if(dataSize) {                      // Non-zero?
            mode              = MODE_DISCARD; // Assume DISCARD until valid
            bytesToDiscard    = dataSize;     // May override below
            minBytesToProcess = 512;          // Hopefully keeps us ahead of
            maxBytesToProcess = 1536;          // 1536 byte payloads @ 60 Hz
            if(minBytesToProcess > dataSize) minBytesToProcess = dataSize;
            if(maxBytesToProcess > dataSize) maxBytesToProcess = dataSize;
            if(headerBuf[0] <= 1) {    // Valid channel?
              if(headerBuf[1] == 0) {  // Valid command? (0 = pixel data)
                mode      = MODE_DATA; // YAY!
                bytesRead = 0;                    // Reset read counter
                if(dataSize <= (NUM_LEDS*3)) {    // <= NUM_LEDS
                  bytesToRead     = dataSize;     // Read all
                  bytesToDiscard  = 0;            // Nothing to discard
                } else {                          // > NUM_LEDS
                  bytesToRead     = (NUM_LEDS*3); // Read first NUM_LEDS
                  bytesToDiscard -= (NUM_LEDS*3); // Discard rest
                }
                nextNumLEDs       = bytesToRead / 3;
              }
            }
          }
        } else if(mode == MODE_DATA) {
          // Read data into bufBeingRead at current bytesRead index
          if((a = client.read(&rgbBuf[bufBeingRead][bytesRead], a)) > 0) {
            bytesToRead -= a;      // Decrement counter by amt actually read
            if(bytesToRead <= 0) { // Done?
              // END OF PIXEL DATA.  Record arrival time and interval since
              // last frame, advance buffer indices, switch to HEADER mode
              // if end of packet, else to DISCARD mode for any remainders.
              uint32_t t          = micros();
              timeBetweenFrames   = t - lastFrameTime;
              lastFrameTime       = t;
              bufPrior            = bufMostRecentlyRead;
              bufMostRecentlyRead = bufBeingRead;
              bufBeingRead        = (bufBeingRead + 1) % 3;
              numLEDs             = nextNumLEDs;
              if(bytesToDiscard) {
                mode              = MODE_DISCARD;
                minBytesToProcess = 512;
                maxBytesToProcess = 1536;
             } else {
                mode = MODE_HEADER;
                minBytesToProcess = maxBytesToProcess = 4;
              }
            } else { // Not done yet
              bytesRead += a; // Advance bytes-read counter
              // Limit upper size of reads to actual remaining bytes
              if(minBytesToProcess>bytesToRead) minBytesToProcess=bytesToRead;
              if(maxBytesToProcess>bytesToRead) maxBytesToProcess=bytesToRead;
            }
          }
        } else { // MODE_DISCARD
          int16_t b;
          while((a > 0) && ((b = client.read(headerBuf, sizeof(headerBuf))) > 0)) {
            a              -= b; // Decrement counters by
            bytesToDiscard -= b; // amount actually read
          }
          if(bytesToDiscard <= 0) { // Done?  Switch to HEADER mode.
            mode = MODE_HEADER;
            minBytesToProcess = maxBytesToProcess = 4;
          } else {
            // Limit upper size of reads to actual remaining bytes
            if(minBytesToProcess>bytesToRead) minBytesToProcess=bytesToRead;
            if(maxBytesToProcess>bytesToRead) maxBytesToProcess=bytesToRead;
          }
        }
      }
    }
    client.stop();
    Serial.println("client disonnected");
    memset(rgbBuf, 0, sizeof(rgbBuf));
    numLEDs = nextNumLEDs = 512;
    magic(rgbBuf[0], rgbBuf[0], 0, spiBuffer[spiBufferBeingFilled], numLEDs);
    spiBufferBeingFilled = 1 - spiBufferBeingFilled;
  }
}
