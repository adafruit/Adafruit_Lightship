// Open Pixel Control "video" example -- loops an animation
// (MOV, MP4, etc.) for display on an LED matrix (16x16 as
// written here, but easily changed).  Seems to work OK in
// Processing 2.2.1, other versions may put up a struggle.

import processing.video.*;

OPC    opc         = new OPC(this, "192.168.0.60", 7890);
int    arrayWidth  = 16, // Width of LED matrix
       arrayHeight = 16, // Height of LED matrix
       scale       = 15, // Preview window pixel size
       numPixels   = arrayWidth * arrayHeight; // Total # of pixels
PImage img         = new PImage(arrayWidth, arrayHeight, RGB);
Movie  movie;

void setup() {
  size(arrayWidth * scale, arrayHeight * scale, JAVA2D);
  opc.setPixel(numPixels-1, 0); // Alloc pixel array ASAP
  this.registerMethod("dispose", this);
  noSmooth();
  frameRate(30);

  // Set up OPC pixel grid.  Arguments are: 1st pixel index,
  // row length, # of rows, center x, y, horizontal & vertical
  // pixel spacing, angle (radians), 'zigzag' flag (true/false):
  opc.ledGrid(0, arrayWidth, arrayHeight, (width - 1) / 2,
    (height - 1) / 2, scale, scale, 0, true);

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
  opc.enable();       // File write enabled only when movie's playing
  PImage t = m.get(); // Movie frame to temporary PImage
  // First, scale image to a smaller size that maintains
  // aspect ratio, while minor axis fills LED array.
  float  xScale = (float)t.width  / (float)arrayWidth,
         yScale = (float)t.height / (float)arrayHeight;
  if(xScale >= yScale) t.resize(0, arrayHeight);
  else                 t.resize(arrayWidth, 0);
  // Then clip out the center section (whatever fits the LED array)
  // into a new image.  Using a temporary interim image (above) and
  // then copying to global 'img' ensures the scaled-down image
  // is used in draw().  Else a race condition occurs where an
  // unscaled instance would occasionally make it through and cause
  // a visual glitch.
  img.copy(t, (t.width-arrayWidth)/2, (t.height-arrayHeight)/2,
    arrayWidth, arrayHeight, 0, 0, arrayWidth, arrayHeight);
  // Other reason it's done in two steps (instead of using
  // resize capability of img.copy()) is that resize() provides
  // a higher-quality scaling function.
}

// Issue LEDs-off packet when certain exit conditions are caught
void dispose() {
  for(int i=0; i < numPixels; i++) opc.setPixel(i, 0);
  opc.writePixels();
}
