/*
    MPU OSC Teapot Processing demo for MPU6050 DMP modified for OSC
    
    The original demo uses serial port I/O which has been replaced with
    OSC UDP messages in this sketch.
    
    The MPU6050 is connected to an ESP8266 with a battery so it is completely
    wire-free.

    Tested on Processing 3.3.5 running on Ubuntu Linux 14.04

    Dependencies installed using Library Manager

    Note: oscP5 and ToxicLibs libraries are required:
    1. Download both libraries from the corresponding links:
        * Open Sound Control library: https://sojamo.de/libraries/oscP5/
        * Download from https://github.com/postspectacular/toxiclibs/releases
    2. Extract into [userdir]/Documents/Processing/libraries (location may be different on Mac/Linux)
    3. Restart Processing if needed 
 */

import oscP5.*;
import netP5.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;

Quaternion quat = new Quaternion(1, 0, 0, 0);

OscP5 oscP5;

void setup() {
    /* 300px square viewport using OpenGL rendering*/
    size(300, 300, P3D);
    gfx = new ToxiclibsSupport(this);

    /*Setup lights and antialiasing*/
    lights();
    smooth();

    /* Start oscP5, listening for incoming messages at port 9999 */
    oscP5 = new OscP5(this, 9999);

    oscP5.plug(this, "imu", "/imuquat");
}

/* Incoming OSC messages are forwarded to the oscEvent method. */
void oscEvent(OscMessage theOscMessage) {
  /* Print the address pattern and the type tag of the received OscMessage */
  
  /*
  print("### received an osc message.");
  print(" addrpattern: "+theOscMessage.addrPattern());
  println(" typetag: "+theOscMessage.typetag());
  */
}

public void imu(float quant_w, float quant_x, float quant_y, float quant_z) {
  //println(quant_w, quant_x, quant_y, quant_z);
  quat.set(quant_w, quant_x, quant_y, quant_z);
}

void draw() {
    background(0);  //Black background

    pushMatrix();
    translate(width / 2, height / 2);   //Translate everything to the middle of the viewport

    /* 3-step rotation from yaw/pitch/roll angles (gimbal lock!)*/
    //rotateY(-ypr[0]);
    //rotateZ(-ypr[1]);
    //rotateX(-ypr[2]);

    /*  ToxicLibs direct angle/axis rotation from quaternion (NO gimbal lock!)
        (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
        different coordinate system orientation assumptions between Processing
        and InvenSense DMP)
    */
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);
    
    /*Draw main body in red*/
    fill(255, 0, 0, 200);   
    box(10, 10, 200);

    /*Draw front-facing tip in blue*/
    fill(0, 0, 255, 200);   
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();

    /*Draw wings and tail fin in green*/
    fill(0, 255, 0, 200);       
    beginShape(TRIANGLES);
    vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);  //Wing top layer
    vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);  //Wing bottom layer
    vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);  //Tail left layer
    vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);  //Tail right layer
    endShape();
    beginShape(QUADS);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    endShape();

    popMatrix();
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();

    /*If it is not a cone, draw the circular top cap*/
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);

        /*Center point*/
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }

    /*If it is not a cone, draw the circular bottom cap*/
    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);

        // Center point
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
}
