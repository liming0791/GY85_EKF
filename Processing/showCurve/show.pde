// Graphing sketch


// This program takes ASCII-encoded strings
// from the serial port at 9600 baud and graphs them. It expects values in the
// range 0 to 1023, followed by a newline, or newline and carriage return

// Created 20 Apr 2005
// Updated 24 Nov 2015
// by Tom Igoe
// This example code is in the public domain.

import processing.serial.*;

Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph
float[] R = new float[7];
float[] RL = {-1,-1,-1,-1,-1,-1,-1 };

void setup () {
  // set the window size:
  size(800, 600);

  // List all the available serial ports
  // if using Processing 2.1 or later, use Serial.printArray()

  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, "/dev/ttyACM1", 9600);

  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');

  // set inital background:
  background(0);
}
void draw () {
  // draw the line:
  stroke(0, 255, 0);
  line(xPos-1, RL[0], xPos, R[0] );
  stroke(255, 0, 0);
  line(xPos-1, RL[1], xPos, R[1] );
  stroke(0, 0, 255);
  line(xPos-1, RL[2], xPos, R[2] );
  stroke(0, 122, 0);
  line(xPos-1, RL[3], xPos, R[3] );
  stroke(122, 0, 0);
  line(xPos-1, RL[4], xPos, R[4] );
  stroke(0, 0, 122);
  line(xPos-1, RL[5], xPos, R[5] );
  stroke(0, 255, 255);
  line(xPos-1, RL[6], xPos, R[6] );
  
  for(int i = 0; i < 7; i++){
    RL[i] = R[i];
  }

  // at the edge of the screen, go back to the beginning:
  if (xPos >= width) {
    xPos = 0;
    background(0);
  } else {
    // increment the horizontal position:
    xPos++;
  }
}


void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null && inString.length() > 0) {
    String [] inputStringArr = split(inString, ",\t");
    if (inputStringArr.length >= 1) { // q1,q2,q3,q4,rn so we have 5 elements
      R[0] = float(inputStringArr[0]);
      R[1] = float(inputStringArr[1]);
      R[2] = float(inputStringArr[2]);
      R[3] = float(inputStringArr[3]);
      R[4] = float(inputStringArr[4]);
      R[5] = float(inputStringArr[5]);
      R[6] = float(inputStringArr[6]);
    }
  }
  
  print(R[0]);
  print(",");
  print(R[1]);
  print(",");
  print(R[2]);
  print(",");
  print(R[3]);
  print(",");
  print(R[4]);
  print(",");
  print(R[5]);
  print(",");
  println(R[6]);
    
  R[0] = map(R[0], -180, 180, height, 0);
  R[1] = map(R[1], -180, 180, height, 0);
  R[2] = map(R[2], -180, 180, height, 0);
  R[3] = map(R[3], -180, 180, height, 0);
  R[4] = map(R[4], -180, 180, height, 0);
  R[5] = map(R[5], -180, 180, height, 0);
  R[6] = map(R[6], -180, 180, height, 0);

}