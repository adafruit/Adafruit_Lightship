// Open Pixel Control "strandtest" example -- cycles R/G/B
// chaser along LED strip.

OPC opc       = new OPC(this, "192.168.0.60", 7890);
int numPixels = 256; // Set this to actual strand length

void setup() {
  opc.setPixel(numPixels-1, 0); // Alloc pixel array ASAP
  opc.enable();
  this.registerMethod("dispose", this);
  frameRate(30);
}

int   i = numPixels * 2; // Intentionally start off end, these
color c = 0;             // will reset on first draw() pass.

void draw() {
  // Increment position...if end reached, wrap around, next color
  if(++i >= numPixels+4) {
    i = 0;
    if((c >>= 8) == 0) c = 0xFF0000; // R->G->B->R->G->B
  }

  opc.setPixel(i, c);              // Set current pixel
  if(i >= 4) opc.setPixel(i-4, 0); // Clear pixel 4 steps back
  opc.writePixels();
}

// Issue LEDs-off packet when certain exit conditions are caught
void dispose() {
  for(int i=0; i < numPixels; i++) opc.setPixel(i, 0);
  opc.writePixels();
}
