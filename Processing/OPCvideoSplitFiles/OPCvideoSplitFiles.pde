// This is a Processing 2.2.1 sketch specifically for the Dauntless Gauntlets
// guide.  It splits a video into multiple rectangular sections (stacked
// vertically or horizontally) and outputs each to a separate .OPC file to be
// copied to a corresponding gauntlet's SD card.
// This is an EXTREMELY special case and probably very confusing to look at.
// If you're new to the Lightship project, try the standard 'OPCvideo'
// example instead; it makes more sense for general use.

import processing.video.*;

boolean split_x      = false; // If true, split L/R rather than top/bottom
OPC     opc[]        = { new OPC(this, 30, "/Volumes/4GB/L01.opc"),
                         new OPC(this, 30, "/Volumes/4GB/R01.opc") };
int     arrayWidth   = 16, // Width of ONE LED matrix
        arrayHeight  =  5, // Height of ONE LED matrix
        arraysWidth  = arrayWidth  * (split_x ? opc.length : 1), // Width of ALL matrices
        arraysHeight = arrayHeight * (split_x ? 1 : opc.length), // Height of ALL matrices
        scale        = 15, // Preview window pixel size
        numPixels    = arrayWidth * arrayHeight; // # of pixels PER MATRIX
PImage  img          = new PImage(arraysWidth, arraysHeight, RGB);
Movie   movie;

void setup() {
  size(arraysWidth * scale, arraysHeight * scale, JAVA2D);
  for(int i=0; i<opc.length; i++)
    opc[i].setPixel(numPixels-1, 0); // Alloc pixel array ASAP
  this.registerMethod("dispose", this);
  noSmooth();
  frameRate(30);

  // Set up OPC pixel grids.  Arguments are: 1st pixel index,
  // row length, # of rows, center x, y, horizontal & vertical
  // pixel spacing, angle (radians), 'zigzag' flag (true/false):
  for(int i=0; i<opc.length; i++) {
    if(split_x) {
      opc[i].ledGrid(0, arrayWidth, arrayHeight,
        ((width - 1) / (opc.length * 2)) + (i * width / opc.length),
        (height - 1) / 2, scale, scale, 0, true);
    } else {
      opc[i].ledGrid(0, arrayWidth, arrayHeight, (width - 1) / 2,
        ((height - 1) / (opc.length * 2)) + (i * height / opc.length),
        scale, scale, 0, true);
    }
  }

  selectInput("Select a file to process:", "fileSelected");
}

void draw() {
  image(img, 0, 0, width, height);
}

void fileSelected(File selection) {
  if(selection != null) {
    movie = new Movie(this, selection.getAbsolutePath());
    movie.loop();
  } else {
    println("Cancelled");
    exit();
  }
}

void movieEvent(Movie m) {
  m.read();
  for(int i=0; i<opc.length; i++) {
    opc[i].enable();
  }
  PImage t = m.get(); // Movie frame to temporary PImage
  // First, scale image to a smaller size that maintains
  // aspect ratio, while minor axis fills LED array.
  float  xScale = (float)t.width  / (float)arraysWidth,
         yScale = (float)t.height / (float)arraysHeight;
  if(xScale >= yScale) t.resize(0, arraysHeight);
  else                 t.resize(arraysWidth, 0);
  // Then clip out the center section (whatever fits the LED array)
  // into a new image.  Using a temporary interim image (above) and
  // then copying to global 'img' ensures the scaled-down image
  // is used in draw().  Else a race condition occurs where an
  // unscaled instance would occasionally make it through and cause
  // a visual glitch.
  img.copy(t, (t.width-arraysWidth)/2, (t.height-arraysHeight)/2,
    arraysWidth, arraysHeight, 0, 0, arraysWidth, arraysHeight);
  // Other reason it's done in two steps (instead of using
  // resize capability of img.copy()) is that resize() provides
  // a higher-quality scaling function.
}

// Issue LEDs-off packet when certain exit conditions are caught
void dispose() {
  for(int i=0; i < opc.length; i++) {
    for(int j=0; j < numPixels; j++)
      opc[i].setPixel(j, 0);
    opc[i].writePixels();
  }
}
