/*
    MPU Teapot example
    
    This code will display airplane graphs that will follow the MPU6050 movements.

    Use #define OUTPUT_TEAPOT output definition to make this code work.

    Define the serial port in the code below (line 45). Note: Close any other serial instance using the port.

    NOTE:  ToxicLibs library is required.
    1. Download from https://github.com/postspectacular/toxiclibs/releases
    2. Extract into [userdir]/Documents/Processing/libraries (location may be different on Mac/Linux)
    3. Restart Processing if needed 
*/

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

void setup() {
    size(300, 300, OPENGL); // 300px square viewport using OpenGL rendering
    gfx = new ToxiclibsSupport(this);

    /* Setup lights and antialiasing */
    lights();
    smooth();
  
    println(Serial.list()); //Display serial port list for debugging/clarity
    String portName = "COMXX"; //Define the port, port format may be different on Linux/Mac
    port = new Serial(this, portName, 115200); // Open the serial port
    port.write('r'); // Send a single character to trigger DMP init/start
}

void draw() {
    if (millis() - interval > 1000) {
        /*Resend a single character to trigger DMP init/start
        in case the MPU is halted/reset while the applet is running*/
        port.write('r');
        interval = millis();
    }
    
    background(0);  // Black background
    pushMatrix();
    translate(width / 2, height / 2);   //Translate everything to the middle of the viewport

    /* Toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    different coordinate system orientation assumptions between Processing
    and InvenSense DMP)*/
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);

    /*Drawing the airplane*/
    fill(255, 0, 0, 200);   //Draw main body in red
    box(10, 10, 200);
    fill(0, 0, 255, 200);   //Draw front-facing tip in blue
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();
    fill(0, 255, 0, 200);   //Draw wings and tail fin in green
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

void serialEvent(Serial port) {
    interval = millis();
    while (port.available() > 0) {
        int ch = port.read();

        if (synced == 0 && ch != '$') return;   //Initial synchronization - also used to resync/realign if needed
        synced = 1;
        print ((char)ch);

        if ((serialCount == 1 && ch != 2)
            || (serialCount == 12 && ch != '\r')
            || (serialCount == 13 && ch != '\n'))  {
            serialCount = 0;
            synced = 0;
            return;
        }

        if (serialCount > 0 || ch == '$') {
            teapotPacket[serialCount++] = (char)ch;
            if (serialCount == 14) {
                serialCount = 0; // restart packet byte position
                
                /*Get Quaternion from data packet*/
                q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
                q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
                q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
                q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
                for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
                
                quat.set(q[0], q[1], q[2], q[3]);   //Set our ToxicLibs quaternion to new data
            }
        }
    }
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
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
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
