/*
Yay for oscilloscopes.
OK, now that I'm measuring frame rate correctly...

getting about 175 updates/sec with interpolation, dithering and parallel output.
There's a periodic slow moment, I think when a new data packet arrives.
Actual throughput is more like 60-200 updates/sec.

Without parallel output, is potentially up to ~500 updates/sec for interp and dithering.
(actual 70-510)
8 MHz to 512 DotStars
512 * 32 = 16384 bits (plus 32-bit header and 256-bit footer) = 16672 bits
8 MHz / 16672 = 479 updates/sec
If using SPI DMA (not added yet), we'll get this time "free."

If just dithering (no interp), getting 72-800 updates/sec

The good news: runs 'smoothly,' as in frame arrive without fits and starts.
The not-so-good news: 48 MHz M0 is just too slow for interpolation + dithering + parallel output.
We MIGHT be able to pick one or two (e.g. interp + dither, then use SPI DMA for output
on a SINGLE strand), or may have to pare back further (e.g. no interp, but maybe dither,
or other way 'round).
*/


#include <SPI.h>
#include <WiFi101.h>
#include <Adafruit_ZeroDMA.h>
#include <Adafruit_ASFcore.h>
#include "utility/dmac.h"
#include "utility/dma.h"

#define Serial SerialUSB

// CONFIG & GLOBALS --------------------------------------------------------

char *ssid = "NETWORK_NAME",
     *pass = "NETWORK_PASSWORD";

Adafruit_ZeroDMA myDMA;

#define NUM_LEDS        512
#define SPI_BUFFER_SIZE (4 + NUM_LEDS * 4 + ((NUM_LEDS / 2) + 7) / 8)
// SPI buffer includes space for 32-bit '0' header, 32 bits per LED,
// and footer of 1 bit per 2 LEDs (rounded to next byte boundary, for SPI).
// For 512 pixels, that's 2084 bytes per SPI buffer (there's 2).

// Two equal-size SPI buffers are allocated; one's being filled with new
// data as the other's being issued via DMA.
uint8_t spiBuffer[2][SPI_BUFFER_SIZE];
uint8_t spiBufferBeingFilled = 0;    // Index of currently-calculating buf
volatile bool spiReady       = true; // True when SPI DMA ready for new data

// These tables (computed at runtime) are used for gamma correction and
// dithering.  RAM used = 256*9+NUM_LEDS*3 bytes.  512 LEDs = 3840 bytes.
uint8_t loR[256], hiR[256], fracR[256], errR[NUM_LEDS],
        loG[256], hiG[256], fracG[256], errG[NUM_LEDS],
        loB[256], hiB[256], fracB[256], errB[NUM_LEDS];

// Used for interpolation.  512 LEDs = 1536 bytes.
uint8_t rgbMix[NUM_LEDS * 3];

// Data for most-recently-received OPC color payload, payload before that,
// and new in-progress payload currently arriving.  512 LEDs = 4608 bytes.
uint8_t rgbBuf[3][NUM_LEDS * 3];

// Order of color bytes as issued to the DotStar LEDs.
// Current LEDs use BRG order (blue=first, red=second, green=last)
#define DOTSTAR_GREENBYTE 0
#define DOTSTAR_BLUEBYTE  1
#define DOTSTAR_REDBYTE   2
// Pre-2015 DotStars use GBR order.
// THESE VALUES RELATE ONLY TO THE LED STRIP; OPC data is always RGB order.

WiFiServer server(7890); // arg = port to listen on


// UTILITY FUNCTIONS -------------------------------------------------------

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
  Serial.println("HELLO HELLO HELLO");

  SPI.begin();

  // Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  // Do stuff while WiFi starts up...

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

  memset(rgbBuf, 0, sizeof(rgbBuf));

  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("WiFi connected");

  // Print the IP address
  Serial.println((IPAddress)WiFi.localIP());

  // Start the server listening for incoming client connections
  server.begin();
  Serial.println("Server listening on port 7890");
}

// -------------------------------------------------------------------------

// This function interpolates between two RGB input buffers, gamma- and
// color-corrects the interpolated result with 16-bit dithering and issues
// the resulting data to the GPIO port.
// Normally this would be a three-step process (interpolate, reformat for
// GPIO, issue to GPIO) but a quirk of the ESP8266 requires 'interleaving'
// the latter two steps to improve performance.  ESP8266 GPIO operations
// are much slower than the CPU frequency and are 'blocking' -- when two
// operations occur on the same port in rapid succession, the second has to
// wait for the first to finish.  This was a performance killer in this
// application, which requires sending data to the DotStar strips quickly
// as possible.  The workaround here is to interleave GPIO and non-GPIO
// operations (the latter are not blocked by GPIO and continue executing).
// Aligning the number of operations for both is why this version of the
// code requires 8 pins of 64 LEDs.  Additionally, two buffers for GPIO
// data are required -- one is being filled with new data while the other
// is being issued as output.  The 'filled' buffer will be issued to the
// strips on the *next* call to this function.
void magic(
 uint8_t  *rgbIn1,    // First RGB input buffer being interpolated
 uint8_t  *rgbIn2,    // Second RGB input buffer being interpolated
 uint8_t   w2,        // Weighting (0-255) of second buffer in interpolation
 uint8_t  *fillBuf) { // SPI data buf being filled
  uint8_t  mix;
  uint16_t weight1, weight2, byteNum, pixelNum, e;

  // First pass: interpolate between rgbIn1 and rgbIn2 buffers (output to
  // global rgbMix buffer), where w2 is the weighting (0-255) of rgbIn2
  // (e.g. 0 = 100% rgbIn1, 255 = 100% rgbIn2, 127 = ~50% each).

  weight2 = (uint16_t)w2 + 1; // 1-256
  weight1 = 257 - weight2;    // 1-256

  for(byteNum = pixelNum = 0; pixelNum < NUM_LEDS; pixelNum++) {
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
    if(e < 256) { // Cumulative error below threshold
      rgbMix[byteNum++] = loR[mix]; // Use dimmer color
    } else {      // Cumulative error at or above threshold
      rgbMix[byteNum++] = hiR[mix]; // Use brighter color,
      e                -= 256;      // reduce error value
    }
    errR[pixelNum] = e; // Store modified error term back in buffer

    // Repeat same operations for green...
    mix = (rgbIn1[byteNum] * weight1 + rgbIn2[byteNum] * weight2) >> 8;
    if((e = (fracG[mix] + errG[pixelNum])) < 256) {
      rgbMix[byteNum++] = loG[mix];
    } else {
      rgbMix[byteNum++] = hiG[mix];
      e                -= 256;
    }
    errG[pixelNum] = e;

    // ...and blue...
    mix = (rgbIn1[byteNum] * weight1 + rgbIn2[byteNum] * weight2) >> 8;
    if((e = (fracB[mix] + errB[pixelNum])) < 256) {
      rgbMix[byteNum++] = loB[mix];
    } else {
      rgbMix[byteNum++] = hiB[mix];
      e                -= 256;
    }
    errB[pixelNum] = e;
  }

  // Second pass: reorder rgbMix buffer into spiBuf.
  uint16_t  i;
  for(i=0; i<NUM_LEDS; i++) {
    fillBuf[4 + i * 4] = 0xFF;
    fillBuf[4 + i * 4 + 1 + DOTSTAR_REDBYTE]   = rgbMix[i * 3    ];
    fillBuf[4 + i * 4 + 1 + DOTSTAR_GREENBYTE] = rgbMix[i * 3 + 1];
    fillBuf[4 + i * 4 + 1 + DOTSTAR_BLUEBYTE]  = rgbMix[i * 3 + 2];
  }

  while(!spiReady);     // Wait for prior SPI DMA transfer to complete
  SPI.endTransaction(); // End prior transaction, start anew...
  myDMA.free();

  // SERMCOM4 == SPI native SERCOM
  myDMA.configure_peripheraltrigger(SERCOM4_DMAC_ID_TX);
  myDMA.configure_triggeraction(DMA_TRIGGER_ACTON_BEAT);
  myDMA.allocate();

  myDMA.setup_transfer_descriptor(
    fillBuf,                          // Source address
    (void *)(&SERCOM4->SPI.DATA.reg), // Dest address
    SPI_BUFFER_SIZE,                  // Data count
    DMA_BEAT_SIZE_BYTE,               // Bytes/halfwords/words
    true,                             // Increment source address
    false);                           // Don't increment dest
  myDMA.add_descriptor();
  myDMA.register_callback(dma_callback);
  myDMA.enable_callback();

  SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
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

// minBytesToProcess is a least amount of data that must be waiting from the
// client before it's read...any less than this and it's ignored, we make
// another pass through magic().  This is because magic() takes a moment to
// run (about 2.5 ms) and we'd quickly fall behind if it was called every
// time a single byte arrived.  Conversely, maxBytesToProcess sets an upper
// limit for each pass, in order to keep the interpolation and dithering
// going (reading full packets would stall that).
uint16_t minBytesToProcess = 4, maxBytesToProcess = 4;
// Both are initially set to 4 because that's the size of an OPC packet
// header, and MODE_HEADER is the startup state.  The values change as we
// switch between different dtates.

// bytesToRead is the number of bytes remaining in MODE_DATA, while
// bytesRead is the number read so far (used as an index into a destination
// buffer).  bytesToDiscard is the number remaining when in MODE_DISCARD.
int16_t bytesToRead, bytesRead, bytesToDiscard;

// There are three LED data buffers: one currently being read (not
// displayed yet), the one most recently fully read (a complete RGB
// pixel payload, currently being interpolated) and the one before that
// (also a full RGB payload, also part of the interpolation).
// These indices keep track of each, and all are incremented whenever a
// full RGB payload is received (resetting back to 0 as necessary).
uint8_t bufPrior = 0, bufMostRecentlyRead = 1, bufBeingRead = 2;

// For interpolation: lastFrameTime is the absolute time (in cycles) when
// the most recent OPC pixel data packet was fully read.  timeBetweenFrames
// is the interval (also in cycles) between lastFrameTime and the frame
// before that.
uint32_t lastFrameTime = 0, timeBetweenFrames = 0;

uint32_t ptime = 0L;
void loop() {
  WiFiClient client = server.available();
  if(client) {
    Serial.println("new client");
    while(client.connected()) {

// Show approx FPS
      uint32_t t = micros();
      Serial.println((1000000L / (t - ptime)));
      ptime = t;


  // Interpolation weight (0-255) is the ratio of the time since last frame
  // arrived to the prior two frames' interval.
  uint32_t timeSinceFrameStart = micros() - lastFrameTime;
  uint8_t  w = (timeSinceFrameStart >= timeBetweenFrames) ? 255 :
               (int)(255.0 * (float)timeSinceFrameStart /
                             (float)timeBetweenFrames);
  // Fixed-point fails, why? 255L * timeSinceFrameStar / timeBetweenFrames;

  magic(rgbBuf[bufPrior], rgbBuf[bufMostRecentlyRead], w,
    spiBuffer[spiBufferBeingFilled]);
  spiBufferBeingFilled = 1 - spiBufferBeingFilled;

      
      int16_t a = client.available(); // How much data awaits?
//      if(a > 0) Serial.println(a);
//      while(a--) Serial.read();
//      continue;
      if(a >= minBytesToProcess) {    // Enough to bother with?
        if(a > maxBytesToProcess) a = maxBytesToProcess; // Yes, but set limit
        if(mode == MODE_HEADER) {     // In header-reading mode...
          client.read(rgbMix, 4);     // 4-bytes (borrow rgbMix to store)
          uint16_t dataSize = (rgbMix[2] << 8) | rgbMix[3]; // Payload bytes
          if(dataSize) {                      // Non-zero?
            mode              = MODE_DISCARD; // Assume DISCARD until validated
            bytesToDiscard    = dataSize;     // May override below
//            minBytesToProcess = 200;          // Hopefully keeps us ahead of
//            maxBytesToProcess = 600;          // 1536 byte payloads @ 60 Hz
            minBytesToProcess = 512;          // Hopefully keeps us ahead of
            maxBytesToProcess = 1536;          // 1536 byte payloads @ 60 Hz
            if(minBytesToProcess > dataSize) minBytesToProcess = dataSize;
            if(maxBytesToProcess > dataSize) maxBytesToProcess = dataSize;
            if(rgbMix[0] <= 1) {       // Valid channel?
              if(rgbMix[1] == 0) {     // Valid command? (0 = pixel data)
                mode      = MODE_DATA; // YAY!
                bytesRead = 0;                // Reset read counter
                if(dataSize <= 1536) {        // 512 pixels
                  bytesToRead     = dataSize; // Read all
                  bytesToDiscard  = 0;        // Nothing to discard
                } else {                      // >512 pixels
                  bytesToRead     = 1536;     // Read first 512
                  bytesToDiscard -= 1536;     // Discard rest
                }
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
              if(bytesToDiscard) {
                mode = MODE_DISCARD;
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
          // rgbMix[] isn't used right now, it can serve as a 'bit bucket'
          if((a = client.read(rgbMix, a)) > 0) {
            bytesToDiscard -= a; // Decrement counter by amount actually read
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
    }
    client.stop();
    Serial.println("client disonnected");
  }
}
