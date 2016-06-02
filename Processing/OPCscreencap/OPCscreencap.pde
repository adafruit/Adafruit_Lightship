import java.awt.*;
import java.awt.image.*;

OPC opc = new OPC(this, "192.168.0.63", 7890);
int arrayWidth  = 16,
    arrayHeight = 16,
    pixelSize   = 20,
    numPixels   = arrayWidth * arrayHeight,
    screenNum   = 0;

// GLOBAL VARIABLES ---- You probably won't need to modify any of this -------

Robot          bot;
Rectangle      dispBounds;
DisposeHandler dh; // For disabling LEDs on exit

// INITIALIZATION ------------------------------------------------------------

void setup() {
  GraphicsEnvironment     ge;
  GraphicsConfiguration[] gc;
  GraphicsDevice[]        gd;

  size(arrayWidth * pixelSize, arrayHeight * pixelSize, JAVA2D);
  noSmooth();
  frameRate(30);

  dh = new DisposeHandler(this);

  // Init screen capture
  ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
  gd = ge.getScreenDevices();
  try {
    bot = new Robot(gd[screenNum]);
  } catch(AWTException e) {
    System.out.println("new Robot() failed");
  }
  gc           = gd[screenNum].getConfigurations();
  dispBounds   = gc[0].getBounds();

  // Set up OPC pixel grid.  Arguments are: 1st pixel index,
  // row length, # of rows, center x, y, horizontal & vertical
  // pixel spacing, angle (radians), 'zigzag' flag (true/false):
  opc.ledGrid(0, arrayWidth, arrayHeight, (width - 1) / 2,
    (height - 1) / 2, pixelSize, pixelSize, 0, true);
  opc.enable();
}

// PER_FRAME PROCESSING ------------------------------------------------------

void draw () {
  BufferedImage img;
  Image         img2;
  PImage        img3;

  img  = bot.createScreenCapture(dispBounds); // Screen cap

// This does a high-quality resize, but is SLOW:
//  img2 = img.getScaledInstance(arrayWidth, arrayHeight, Image.SCALE_SMOOTH);
// Faster but less accurate:
  img2 = img.getScaledInstance(arrayWidth*16, arrayHeight*16, Image.SCALE_FAST);
  img2 = img2.getScaledInstance(arrayWidth, arrayHeight, Image.SCALE_AREA_AVERAGING);

  img3 = new PImage(img2); // Convert Image to PImage
  image(img3, 0, 0, width, height);

//  println(frameRate); // How are we doing?
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
