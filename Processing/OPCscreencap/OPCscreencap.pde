// Screen capture adapted from code by Cedrik Kiefer (processing.org forum)

import java.awt.*;
import java.awt.image.*;

// CONFIGURABLE PROGRAM CONSTANTS --------------------------------------------

OPC opc = new OPC(this, "192.168.0.60", 7890);
int arrayWidth  = 16,
    arrayHeight = 16,
    pixelSize   = 20,
    numPixels   = arrayWidth * arrayHeight;

// Depending on many factors, it may be faster either to capture full
// screens and process only the pixels needed, or to capture multiple
// smaller sub-blocks bounding each region to be processed.  Try both,
// look at the reported frame rates in the Processing output console,
// and run with whichever works best for you.

static final boolean useFullScreenCaps = true;


// PER-DISPLAY INFORMATION ---------------------------------------------------

// This array contains details for each display that the software will
// process.  If you have screen(s) attached that are not among those being
// "Adalighted," they should not be in this list.  Each triplet in this
// array represents one display.  The first number is the system screen
// number...typically the "primary" display on most systems is identified
// as screen #1, but since arrays are indexed from zero, use 0 to indicate
// the first screen, 1 to indicate the second screen, and so forth.  This
// is the ONLY place system screen numbers are used...ANY subsequent
// references to displays are an index into this list, NOT necessarily the
// same as the system screen number.  For example, if you have a three-
// screen setup and are illuminating only the third display, use '2' for
// the screen number here...and then, in subsequent section, '0' will be
// used to refer to the first/only display in this list.
// The second and third numbers of each triplet represent the width and
// height of a grid of LED pixels attached to the perimeter of this display.
// For example, '9,6' = 9 LEDs across, 6 LEDs down.

static final int displays[][] = new int[][] {
   {0,16,16} // Screen 0, 9 LEDs across, 6 LEDs down
//,{1,9,6} // Screen 1, also 9 LEDs across and 6 LEDs down
};

// PER-LED INFORMATION -------------------------------------------------------

// This array contains the 2D coordinates corresponding to each pixel in the
// LED strand, in the order that they're connected (i.e. the first element
// here belongs to the first LED in the strand, second element is the second
// LED, and so forth).  Each triplet in this array consists of a display
// number (an index into the display array above, NOT necessarily the same as
// the system screen number) and an X and Y coordinate specified in the grid
// units given for that display.  {0,0,0} is the top-left corner of the first
// display in the array.
// For our example purposes, the coordinate list below forms a ring around
// the perimeter of a single screen, with a one pixel gap at the bottom to
// accommodate a monitor stand.  Modify this to match your own setup:

static final int leds[][] = new int[256][3];
/*
{
  {0,3,5}, {0,2,5}, {0,1,5}, {0,0,5}, // Bottom edge, left half
  {0,0,4}, {0,0,3}, {0,0,2}, {0,0,1}, // Left edge
  {0,0,0}, {0,1,0}, {0,2,0}, {0,3,0}, {0,4,0}, // Top edge
           {0,5,0}, {0,6,0}, {0,7,0}, {0,8,0}, // More top edge
  {0,8,1}, {0,8,2}, {0,8,3}, {0,8,4}, // Right edge
  {0,8,5}, {0,7,5}, {0,6,5}, {0,5,5}  // Bottom edge, right half

/* Hypothetical second display has the same arrangement as the first.
   But you might not want both displays completely ringed with LEDs;
   the screens might be positioned where they share an edge in common.
 ,{1,3,5}, {1,2,5}, {1,1,5}, {1,0,5}, // Bottom edge, left half
  {1,0,4}, {1,0,3}, {1,0,2}, {1,0,1}, // Left edge
  {1,0,0}, {1,1,0}, {1,2,0}, {1,3,0}, {1,4,0}, // Top edge
           {1,5,0}, {1,6,0}, {1,7,0}, {1,8,0}, // More top edge
  {1,8,1}, {1,8,2}, {1,8,3}, {1,8,4}, // Right edge
  {1,8,5}, {1,7,5}, {1,6,5}, {1,5,5}  // Bottom edge, right half
*/

// GLOBAL VARIABLES ---- You probably won't need to modify any of this -------

short[][]        ledColor    = new short[leds.length][3];
int              nDisplays   = displays.length;
Robot[]          bot         = new Robot[displays.length];
Rectangle[]      dispBounds  = new Rectangle[displays.length],
                 ledBounds;  // Alloc'd only if per-LED captures
int[][]          pixelOffset = new int[leds.length][256],
                 screenData; // Alloc'd only if full-screen captures
PImage[]         preview     = new PImage[displays.length];
DisposeHandler   dh; // For disabling LEDs on exit

// INITIALIZATION ------------------------------------------------------------

void setup() {
  GraphicsEnvironment     ge;
  GraphicsConfiguration[] gc;
  GraphicsDevice[]        gd;
  int                     d, i, totalWidth, maxHeight, row, col, rowOffset;
  int[]                   x = new int[16], y = new int[16];
  float                   f, range, step, start;

  dh = new DisposeHandler(this); // Init DisposeHandler ASAP

i = 0;
for(row=0; row<arrayHeight; row++) {
  for(col=0; col<arrayWidth; col++) {
    leds[i][0] = 0;
leds[i][1] = col;
leds[i][2] = row;
i++;
  }
}

  // Initialize screen capture code for each display's dimensions.
  dispBounds = new Rectangle[displays.length];
  if(useFullScreenCaps == true) {
    screenData = new int[displays.length][];
    // ledBounds[] not used
  } else {
    ledBounds  = new Rectangle[leds.length];
    // screenData[][] not used
  }
  ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
  gd = ge.getScreenDevices();
  if(nDisplays > gd.length) nDisplays = gd.length;
  totalWidth = maxHeight = 0;
  for(d=0; d<nDisplays; d++) { // For each display...
    try {
      bot[d] = new Robot(gd[displays[d][0]]);
    }
    catch(AWTException e) {
      System.out.println("new Robot() failed");
      continue;
    }
    gc              = gd[displays[d][0]].getConfigurations();
    dispBounds[d]   = gc[0].getBounds();
    dispBounds[d].x = dispBounds[d].y = 0;
    preview[d]      = createImage(displays[d][1], displays[d][2], RGB);
    preview[d].loadPixels();
    totalWidth     += displays[d][1];
    if(d > 0) totalWidth++;
    if(displays[d][2] > maxHeight) maxHeight = displays[d][2];
  }

  // Precompute locations of every pixel to read when downsampling.
  // Saves a bunch of math on each frame, at the expense of a chunk
  // of RAM.  Number of samples is now fixed at 256; this allows for
  // some crazy optimizations in the downsampling code.
  for(i=0; i<leds.length; i++) { // For each LED...
    d = leds[i][0]; // Corresponding display index

    // Precompute columns, rows of each sampled point for this LED
    range = (float)dispBounds[d].width / (float)displays[d][1];
    step  = range / 16.0;
    start = range * (float)leds[i][1] + step * 0.5;
    for(col=0; col<16; col++) x[col] = (int)(start + step * (float)col);
    range = (float)dispBounds[d].height / (float)displays[d][2];
    step  = range / 16.0;
    start = range * (float)leds[i][2] + step * 0.5;
    for(row=0; row<16; row++) y[row] = (int)(start + step * (float)row);

    if(useFullScreenCaps == true) {
      // Get offset to each pixel within full screen capture
      for(row=0; row<16; row++) {
        for(col=0; col<16; col++) {
          pixelOffset[i][row * 16 + col] =
            y[row] * dispBounds[d].width + x[col];
        }
      }
    } else {
      // Calc min bounding rect for LED, get offset to each pixel within
      ledBounds[i] = new Rectangle(x[0], y[0], x[15]-x[0]+1, y[15]-y[0]+1);
      for(row=0; row<16; row++) {
        for(col=0; col<16; col++) {
          pixelOffset[i][row * 16 + col] =
            (y[row] - y[0]) * ledBounds[i].width + x[col] - x[0];
        }
      }
    }
  }

  // Preview window shows all screens side-by-side
  size(totalWidth * pixelSize, maxHeight * pixelSize, JAVA2D);
  noSmooth();
  frameRate(30);
}

// PER_FRAME PROCESSING ------------------------------------------------------

void draw () {
  BufferedImage img;
  int           d, i, j, o, c, weight, rb, g, sum, deficit, s2;
  int[]         pxls, offs;

  if(useFullScreenCaps == true ) {
    // Capture each screen in the displays array.
    for(d=0; d<nDisplays; d++) {
      img = bot[d].createScreenCapture(dispBounds[d]);
      // Get location of source pixel data
      screenData[d] =
        ((DataBufferInt)img.getRaster().getDataBuffer()).getData();
    }
  }

  // This computes a single pixel value filtered down from a rectangular
  // section of the screen.  While it would seem tempting to use the native
  // image scaling in Processing/Java, in practice this didn't look very
  // good -- either too pixelated or too blurry, no happy medium.  So
  // instead, a "manual" downsampling is done here.  In the interest of
  // speed, it doesn't actually sample every pixel within a block, just
  // a selection of 256 pixels spaced within the block...the results still
  // look reasonably smooth and are handled quickly enough for video.

  for(i=0; i<leds.length; i++) {  // For each LED...
    d = leds[i][0]; // Corresponding display index
    if(useFullScreenCaps == true) {
      // Get location of source data from prior full-screen capture:
      pxls = screenData[d];
    } else {
      // Capture section of screen (LED bounds rect) and locate data::
      img  = bot[d].createScreenCapture(ledBounds[i]);
      pxls = ((DataBufferInt)img.getRaster().getDataBuffer()).getData();
    }
    offs = pixelOffset[i];
    rb = g = 0;
    for(o=0; o<256; o++) {
      c   = pxls[offs[o]];
      rb += c & 0x00ff00ff; // Bit trickery: R+B can accumulate in one var
      g  += c & 0x0000ff00;
    }

    opc.setPixel(i, ((rb & 0xff00ff00) | (g & 0x00ff0000)) >> 8);

    ledColor[i][0]  = (short)((rb >> 24) & 0xff);
    ledColor[i][1]  = (short)(( g >> 16) & 0xff);
    ledColor[i][2]  = (short)((rb >>  8) & 0xff);

    // Update pixels in preview image
//    preview[d].pixels[leds[i][2] * displays[d][1] + leds[i][1]] =
    preview[0].pixels[i] =
     (ledColor[i][0] << 16) | (ledColor[i][1] << 8) | ledColor[i][2];
  }

  opc.writePixels();

  // Show live preview image(s)
  scale(pixelSize);
  for(i=d=0; d<nDisplays; d++) {
    preview[d].updatePixels();
    image(preview[d], i, 0);
    i += displays[d][1] + 1;
  }

  println(frameRate); // How are we doing?
}

public class DisposeHandler { // LEDs off when exiting
  DisposeHandler(PApplet pa) {
    pa.registerMethod("dispose", this);
  }
  public void dispose() {
    for(int i=0; i < numPixels; i++) {
      opc.setPixel(i, 0);
    }
    opc.writePixels();
  }
}
