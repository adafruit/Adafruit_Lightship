// Open Pixel Control "piano roll" example -- provides a
// rudimentary animation editor/sequencer by using a cross-
// section of a vertically-scrolling image to set pixel colors.

OPC    opc       = new OPC(this, "192.168.0.60", 7890);
int    numPixels = 256,    // Length of LED strip
       xPos      = 0,      // Scrolling image X position
       yPos      = 0x7FFF, // Scrolling image Y position
       speed     = 30;     // Frame rate
PImage img;

void setup() {
  size(numPixels, 140, P2D);
  opc.setPixel(numPixels-1, 0); // Alloc pixel array ASAP
  this.registerMethod("dispose", this);
  frameRate(speed);
  textSize(12);
  textAlign(CENTER);
  selectInput("Select a file to process:", "fileSelected");
}

void draw() {
  background(0);          // Clear screen
  if(img == null) return; // If no image loaded yet, do nothing.

  image(img, xPos, yPos); // Draw image at current position
  // Advance Y by 1 pixel...if off top, reset to bottom
  if(--yPos < -img.height) yPos = height;

  // Draw FPS indicator
  fill(30, 30, 30);    // Slider bar background
  noStroke();
  rect(0, 120, width, 20);
  fill(100, 100, 100); // Slider bar foreground
  stroke(255, 255, 255);
  rect(0, 120, width * speed / 60 - 1, 19);
  String s = speed + " FPS";
  int w = (int)textWidth(s) + 8;
  noStroke();
  fill(0, 0, 0, 128);  // Text background
  rect((width - w) / 2, 123, w, 14);
  fill(255, 255, 255); // Text foreground
  text(s, width/2, height - 6);
}

void fileSelected(File selection) {
  if(selection != null) {
    img      = loadImage(selection.getAbsolutePath());
    xPos     = (width - img.width) / 2;
    yPos     = height;
    opc.ledStrip(0, numPixels, (width - 1) / 2, 60, 1, 0.0, false);
    opc.enable(); // File write enabled after image selected
  } else {
    println("Cancelled");
    exit();
  }
}

void mouseClicked() { setSpeed(); }
void mouseDragged() { setSpeed(); }

void setSpeed() {
  if(mouseY >= 120) {
    speed = 1 + 59 * mouseX / width;
    if(speed < 1)       speed = 1;
    else if(speed > 60) speed = 60;
    frameRate(speed);
  }
}

// Issue LEDs-off packet when certain exit conditions are caught
void dispose() {
  for(int i=0; i < numPixels; i++) opc.setPixel(i, 0);
  opc.writePixels();
}
