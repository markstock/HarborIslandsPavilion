//
// HarborIslandsPavilion
//
// (c) 2014-2015 Mark J Stock
//
// v.8a uses 2D shallow water equations to generate waves
//      and verlet integrator for wind-blown grass blades
//      while integrating live weather
//      corrections include safer XML parsing
//
// Some portions (c) 2014 Brian Knep
//

// We need this for the setUndecorated-like calls
import processing.awt.PSurfaceAWT;

// Query live weather?
boolean useLiveWeather = false;

// fluid part

// Simulation domain is [0:nx)[0:ny)
//   and is periodic in x and y
//   nx and ny already include 1 buffer cell on all sides
// 128^2 can run at 20fps on 2.3GHz laptop
int nx = 32;
int ny = nx;
float[][] hp1 = new float[nx][ny];
float[][] h = new float[nx][ny];
float[][] hm1 = new float[nx][ny];
float[][] d2hdt2 = new float[nx][ny];
float c1 = 10.0f;  // wave speed (g*d)
float c2 = 0.03f;  // diffusion (1/D)
// rows to render
int nRows = 4;
int[] row = new int[nRows];
float drawMag = 0.3f;

float fdt = 0.003f;

// grass part

// How many blades of grass
int nBlades = 24;
// How many segments in each blade?
int nNodes = 20;
float[][][] s = new float[nBlades][nNodes][2];

// segment length for each blade
float[] dl = new float[nBlades];
// position of base of blade
float[] xc = new float[nBlades];
float[] zc = new float[nBlades];
// angles and accel of each joint
float[][] ap1 = new float[nBlades][nNodes];
float[][] a   = new float[nBlades][nNodes];
float[][] am1 = new float[nBlades][nNodes];
float[][] zta = new float[nBlades][nNodes];
float[][] da  = new float[nBlades][nNodes];
float[][] dda = new float[nBlades][nNodes];
// stiffness of every joint
float[][] kk = new float[nBlades][nNodes];

// fluctuations in wind speed
int nNums = 500;
float[] v = new float[nNums];
float denom = sqrt(float(nNums));

float gdt = 0.01f;


// The zip code we'll check for
String zip = "02045";    // this is Hull, MA
int thisHour = 0;
int thisMin = 0;
int millisSinceMinute = 0;
int fadeMillis = 5000;

// Set default wind
float speed = 1.0f;
float direction = PI / 2.0f;
float[] wind = new float[2];

void getWeatherTesting() {
  // test values
  speed = 15.f;
  direction = 270.f * (PI/180.f);
  
  // set up wind vector
  wind[0] = speed * cos(direction);
  wind[1] = speed * sin(direction);
}

void getWeather() {
  // The URL for the XML document
  String url = "http://xml.weather.yahoo.com/forecastrss?p=" + zip;
  
  // Load the XML document
  XML xml = loadXML(url);

  // Get the wind data element
  try {
    XML windData = xml.getChild("channel").getChild("yweather:wind");
    if (windData.hasAttribute("direction")) {
      direction = 2.f * PI * windData.getInt("direction") / 360.f;
    }
    if (windData.hasAttribute("speed")) {
      speed = windData.getInt("speed");
    }
  } catch (Exception e) {
    // reuse old values for speed and direction and continue
  }

  // set up wind vector
  wind[0] = speed * cos(direction);
  wind[1] = speed * sin(direction);
}

void setupWaves() {
  // set up the heightfield
  println("Fluid domain is "+nx+" "+ny);
  
  // water height field
  for (int i=0; i<nx; i++) {
  for (int j=0; j<ny; j++) {
    h[i][j] = 2.f * float(i+j)/float(nx+ny);
    hm1[i][j] = h[i][j];
  }
  }
}

void setupGrass() {
  // set up the blades
  for (int i=0; i<nBlades; i++) {
    xc[i] = random(float(width)/7, 6*float(width)/7);
    zc[i] = (255*i)/nBlades + random(0, 255/nBlades);
    float ylen = 0.5*float(height) + 0.4*random(float(height));
    dl[i] = ylen / float(nNodes-2);
    float thisStiff = 2.0f + random(4.0f);
    float thisLean = random(-0.06f,0.06f) * PI;
    for (int j=0; j<nNodes; j++) {
      // note that first and last nodes are not drawn,
      //   nor are the segments drawn between them
      // node [1] is at the floor, [nNodes-2] is the tip
      // zero-torque angle
      zta[i][j] = 2.0f * thisLean / float(nNodes);
      //zta[i][j] = 0.0f;
      // current angle (relative to previous segment)
      a[i][j] = zta[i][j];
      // previous angle
      am1[i][j] = a[i][j];
      // angular acceleration
      dda[i][j] = 0.0f;
      // angular velocity
      da[i][j] = 0.0f;
      // stiffness
      kk[i][j] = thisStiff;// * (2*nNodes-j)/float(nNodes);
    }
    // just kidding---the first angle needs to be non-zero
    a[i][0] = 0.5f * PI + thisLean;
    am1[i][0] = a[i][0];
  }
  
  // find velocity and position of blades from angles
  for (int i=0; i<nBlades; i++) {
    // first node is under "ground"
    s[i][0][0] = xc[i];
    s[i][0][1] = float(height) + dl[i];
    // start running angular sum
    float aSum = 0.0f;
    for (int k=1; k<nNodes; k++) {
      aSum += a[i][k-1];
      s[i][k][0] = s[i][k-1][0] + dl[i]*cos(aSum);
      s[i][k][1] = s[i][k-1][1] - dl[i]*sin(aSum);
    }
  }
}

// This stuff used to be in init(), but is now in setup()
// see https://forum.processing.org/two/discussion/12260/processing-3-init-disappearance
//void init() {
//  // These help ensure that the frame stays on top
//  frame.removeNotify();
//  frame.setUndecorated(true);
//  frame.setAlwaysOnTop(true); 
//  frame.addNotify();
//  super.init();
//}

void setup() {
  // for testing, use a larger size
  //size(256, 192);
  // for production, use these sizes
  size(64, 48);
  
  frameRate(60);
  noStroke();
  smooth();
  curveDetail(3);
  hint(ENABLE_STROKE_PURE);  // this is amazing!
  
  PSurfaceAWT awtSurface = (PSurfaceAWT)surface;
  PSurfaceAWT.SmoothCanvas smoothCanvas = (PSurfaceAWT.SmoothCanvas)awtSurface.getNative();
  smoothCanvas.getFrame().setAlwaysOnTop(true);
  smoothCanvas.getFrame().removeNotify();
  smoothCanvas.getFrame().setUndecorated(true);
  smoothCanvas.getFrame().setLocation(730, 567);
  smoothCanvas.getFrame().addNotify();

  // grab a wind vector
  if (useLiveWeather) {
    getWeather();
  } else {
    getWeatherTesting();
  }
  println("Wind speed is "+speed);
  
  // and set the current time
  thisHour = hour()*6 + minute()/10;
  thisMin = minute();
  
  // set up the array sum
  for (int i=0; i<nNums; i++) {
    v[i] = random(-1.0f, 1.0f);
  }
  
  setupGrass();
  setupWaves();
}

void drawWaves(int alph) {
  fill(0);
  strokeWeight(width/32);
  float fx = width/float(nx-1);
  float wot = width/2.f;
  float fy = height/float(ny-1);
  //for (int j=0; j<nRows; j++) {
  //for (int j=0; j<ny; j+=ny/14) {
  for (int j=ny/10; j<ny-ny/10; j+=ny/10) {
    stroke((255*j)/ny, alph);
    float scale = 1.0f + 9.0f*float(j)/ny;
    float fxs = scale*fx;
    float dms = drawMag*scale;
    beginShape();
    curveVertex(wot+float(-6-nx/2)*fxs, 70+(j+dms*h[0][j])*fy);
    curveVertex(wot+float(-4-nx/2)*fxs, 25+(j+dms*h[0][j])*fy);
    curveVertex(wot+float(-2-nx/2)*fxs, 5+(j+dms*h[0][j])*fy);
    for (int i=0; i<nx; i++) {
      //curveVertex(float(i)*fxs, (j+dms*h[i][j])*fy);
      curveVertex(wot+float(i-nx/2)*fxs, (j+dms*h[i][j])*fy);
    }
    curveVertex(wot+float(nx/2+1)*fxs, 5+(j+dms*h[nx-1][j])*fy);
    curveVertex(wot+float(nx/2+3)*fxs, 25+(j+dms*h[nx-1][j])*fy);
    curveVertex(wot+float(nx/2+5)*fxs, 70+(j+dms*h[nx-1][j])*fy);
    endShape();
  }
}

void drawGrass(int alph) {
  // grass stuff
  noFill();
  strokeWeight(height/32);
  for (int i=0; i<nBlades; i++) {
    stroke(zc[i], alph);
    beginShape();
    for (int j=0; j<nNodes; j++) {
      curveVertex(s[i][j][0], s[i][j][1]);
    }
    endShape();
  }
}

void draw() {
  // use this for testing larger screen size multiples
  //frame.setLocation(730 - (width-baseWidth)/2, 567 - (height-baseHeight));
  // keep this - this is the true screen location
  frame.setLocation(730, 567);

  // Uncomment this to limit piece to evenings
  // if (hour() > 8 && hour() < 18) {
  //  background(0);
  //   return;
  // }
  
  // recheck for wind speed/direction every 10 minutes
  //if (hour() != thisHour) {
    //thisHour = hour();
  if (hour()*6 + minute()/10 != thisHour) {
    thisHour = hour()*6 + minute()/10;
    if (useLiveWeather) {
      getWeather();
    } else {
      getWeatherTesting();
    }
    println("Wind speed is now "+speed);
  }
  
  // Use the centered-difference scheme as written at
  // http://www.uio.no/studier/emner/matnat/ifi/INF2340/v05/foiler/sim04.pdf
  
  // wave part
  
  // poke/splash at a point
  if (frameCount%10 == 1) {
    //println("splash");
    // for one poke every 10 frames, 0.1 is soft seas, 0.3 is wild
    //h[int(random(nx))][int(random(ny))] -= 0.05f;
    h[int(random(nx))][int(random(ny))] -= 0.015f * speed;
  }
  
  // compute the dhdt term
  for (int i=1; i<nx-1; i++) {
    for (int j=1; j<ny-1; j++) {
      float laplace = 0.25f * (h[i-1][j] + h[i+1][j] + 
                               h[i][j-1] + h[i][j+1] - 
                               4.0*h[i][j]);
      float dhdt = h[i][j] - hm1[i][j];
      d2hdt2[i][j] = c1 * (laplace - c2*dhdt);
    }
  }
  
  // integrate
  float sum = 0.0f;
  for (int i=1; i<nx-1; i++) {
  for (int j=1; j<ny-1; j++) {
    // directly compute new height by central difference in time
    hp1[i][j] = 2.0f*h[i][j] - hm1[i][j] + fdt * d2hdt2[i][j];
    
    // keep running sum
    sum += hp1[i][j];
  }
  }
  // mean elevation (should be 0.0)
  sum /= float((nx-2)*(ny-2));
  
  // copy the boundary/buffer data
  for (int i=1; i<nx-1; i++) {
    hp1[i][0] = hp1[i][ny-2];
    hp1[i][ny-1] = hp1[i][1];
  }
  for (int j=1; j<ny-1; j++) {
    hp1[0][j] = hp1[nx-2][j];
    hp1[nx-1][j] = hp1[1][j];
  }
  hp1[0][0] = hp1[nx-2][ny-2];
  hp1[0][ny-1] = hp1[nx-2][1];
  hp1[nx-1][0] = hp1[1][ny-2];
  hp1[nx-1][ny-1] = hp1[1][1];
  
  // copy the arrays down
  for (int i=0; i<nx; i++) {
  for (int j=0; j<ny; j++) {
    hm1[i][j] = h[i][j];
  }
  }
  for (int i=0; i<nx; i++) {
  for (int j=0; j<ny; j++) {
    h[i][j] = hp1[i][j] - sum;
  }
  }
  
  // find local velocities (dhdt)? need this to generate wind waves
  
  // grass part
  // Update the wind fluctuation summation
  v[frameCount%nNums] = random(-1.0f, 1.0f);
  float fluctsum = 0.0f;
  for (int i=0; i<nNums; i++) {
    fluctsum += v[i];
  }
  fluctsum /= denom;
  //if (frameCount%10 == 1) println("sum is "+fluctsum);
  // scale this 1.0 to adjust the wind speed
  float localWind = 0.1f*speed * (1.0f + fluctsum);
  // flip it based on direction
  if (direction < PI) localWind = -localWind;
  
  // compute blade segment angle wrt horizontal
  for (int i=0; i<nBlades; i++) {
    // skip underground node 0, at-ground node 1,
    //   and ghost tip node nNodes-1
    // use dda to store the sum of the angles up the blade
    dda[i][0] = a[i][0];
    for (int k=1; k<nNodes; k++) {
      dda[i][k] = dda[i][k-1] + a[i][k];
      //if (i==0 && k==nNodes-1) println("angle at "+k+" is "+dda[i][k]);
    }
  }
  
  // accumulate torques down the blade
  for (int i=0; i<nBlades; i++) {
    // no torque on the (invisible) blade tip or root
    dda[i][nNodes-1] = 0.0f;
    dda[i][1] = 0.0f;
    float accumTorque = 0.0f;
    for (int k=nNodes-2; k>1; k--) {
      // apply torque, account for blade angle
      dda[i][k] = dl[i] * 0.01f * localWind * abs(sin(dda[i][k]));
      
      // accumulate external forces down the blade
      accumTorque += dda[i][k];
      dda[i][k] = accumTorque;
      
      // restoring spring force
      dda[i][k] -= kk[i][k]*(a[i][k]-zta[i][k]);
      // damping force
      dda[i][k] -= 0.05f * da[i][k];
    }
  }
  
  // advect the angles
  for (int i=0; i<nBlades; i++) {
    for (int k=2; k<nNodes; k++) {
      ap1[i][k] = 2.0f*a[i][k] - am1[i][k] + gdt * dda[i][k];
      da[i][k] = 0.5f * (ap1[i][k] - am1[i][k]) / gdt;
    }
  }
  
  // copy arrays down
  for (int i=0; i<nBlades; i++) {
    for (int k=2; k<nNodes; k++) {
      am1[i][k] = a[i][k];
      a[i][k] = ap1[i][k];
    }
  }
  
  // find velocity and position of blades from angles
  for (int i=0; i<nBlades; i++) {
    // first node is under "ground", 2nd node is at ground
    float aSum = a[i][0];
    for (int k=2; k<nNodes; k++) {
      aSum += a[i][k-1];
      s[i][k][0] = s[i][k-1][0] + dl[i]*cos(aSum);
      s[i][k][1] = s[i][k-1][1] - dl[i]*sin(aSum);
    }
  }
  
  
  // reset fine timer when minutes changes
  if (minute() != thisMin) {
    thisMin = minute();
    millisSinceMinute = millis();
  }
  float ffrac = float(millis()-millisSinceMinute)/float(fadeMillis);
  int frac = int(255.f*ffrac);
  frac = constrain(frac, 0, 255);

  // draw stuff
  background(0);
  if (minute()%2 == 0) {
    drawWaves(frac);
    drawGrass(255-frac);
    //println("top "+frac+" "+(255-frac));
  } else {
    drawWaves(255-frac);
    drawGrass(frac);
    //println("bot "+frac+" "+(255-frac));
  }
  
  //if (frameCount%60 == 1) println(frameRate+" fps");
}

void keyPressed() {
  println(frameRate+" fps");
}