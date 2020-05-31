import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import SimpleOpenNI.*; 
import processing.serial.*; 
import controlP5.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Kinect_Artificial_Limb extends PApplet {

/*---------------------------------------------------------------------------------------------------------*/
//Load Library//
       


/*---------------------------------------------------------------------------------------------------------*/
//Kinect Definition
SimpleOpenNI  kinect;
 //Port Definition
Serial port; 
 //ControlIP5 Definition
ControlP5 cp5;
/*---------------------------------------------------------------------------------------------------------*/
PImage kinectDepth, kinectRGB;                     // image storage from kinect
PImage BackGround;

PVector headPosition = new PVector();              // postion of head to draw circle

float distanceScalar;                              // turn headPosition into scalar form
float headSize = 100;                              // Head Size Diameter

int userID;
int a; //shoulder forward                             // Define the Initial Angle of the Servo
int b;
int c;
int d;
int e;
int f;

int Length_Large_Arm, Length_Small_Arm;
int Angle_Shoulder_Forward;
int Angle_Shoulder_Out;
int Angle_Elbow;

int start_flag = 0; //Mode Flag
  
byte out[] = new byte[4];                          // Define Port Data Array
/*---------------------------------------------------------------------------------------------------------*/
public void setup()
{
  // Setup the Kinect //
  kinect = new SimpleOpenNI(this);                 // start a new kinect object
  port = new Serial(this,"com3",9600);          // start a new Serial Port
  
  kinect.enableDepth();                            // enable depth sensor
  kinect.enableRGB();                              // enable RGB
  kinect.enableUser();                             // enable skeleton generation for all joints
  kinect.setMirror(true);
  
  // Setup the window //
  size(1280,680);                                 
  smooth();
  strokeWeight(5);
  fill(255, 0, 0);
  textSize(20);
  //  //
  
  // Load File //
  BackGround = loadImage("BackGround.jpg");
  
  // Setup ControlIP5 //
  cp5 = new ControlP5(this);                       // start ControlIP5 
  PFont font = createFont("arial",30);
  
  cp5.addButton("START")
     .setValue(1)
     .setPosition(485,490)
     .setSize(300,50)
     .setFont(font)
     ;
  
  // and add another 2 buttons
  cp5.addButton("Introduction")
     .setValue(2)
     .setPosition(485,550)
     .setSize(300,50)
     .setFont(font)
     ;
     
  cp5.addButton("Back")
     .setValue(0)
     .setPosition(485,610)
     .setSize(300,50)
     .setFont(font)
     ;
}
/*---------------------------------------------------------------------------------------------------------*/
public void draw()
{
  background(255);
  Display_Init();
  kinect.update(); // update the camera
  kinectDepth = kinect.depthImage();
  kinectRGB   = kinect.rgbImage();
  //println(start_flag);

  if(start_flag == 1)
  {
    image(kinectDepth, 0, 0); 
    image(kinectRGB, 640, 0);
    IntVector userList = new IntVector();
    kinect.getUsers(userList);
    if (userList.size()>0)
    {
      userID = userList.get(0);
      if (kinect.isTrackingSkeleton(userID))
      {
        stroke(255, 0, 0);
        fill(255, 0, 0);
        drawSkeleton(userID);                     //Draw the First Skeleton
        fill(255);                                //Color of the Rect
        rect(0,480,1280,680);                     //White Rect at the Base of the Window
        PVector torsoPosition = new PVector();
        PVector leftHand      = new PVector();
        PVector rightHand     = new PVector();
        PVector leftShoulder  = new PVector();
        PVector rightShoulder = new PVector();
        PVector leftElbow     = new PVector();
        PVector rightElbow    = new PVector();
        PVector leftHip       = new PVector();
        PVector rightHip      = new PVector();
        
        kinect.getJointPositionSkeleton(userID, SimpleOpenNI.SKEL_TORSO, torsoPosition); //Get Torso's PVector
        kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_LEFT_HAND, leftHand);    //Get LeftHand's PVector
        kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_RIGHT_HAND, rightHand);  //Get RightHand's PVector
        kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_LEFT_SHOULDER, leftShoulder);  //Get LeftShoulder's PVector
        kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_RIGHT_SHOULDER, rightShoulder);  //Get RightShoulder's PVector
        kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_LEFT_ELBOW, leftElbow);  //Get LeftElbow's PVector
        kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_RIGHT_ELBOW, rightElbow);  //Get RightElbow's PVector
        kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_LEFT_HIP, rightHip);  //Get LeftHip's PVector
        kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_RIGHT_HIP, rightHip);  //Get RightHip's PVector
        
        PVector rightHand2D_XY     = new PVector(rightHand.x, rightHand.y);
        PVector rightElbow2D_XY    = new PVector(rightElbow.x, rightElbow.y);
        PVector rightShoulder2D_XY = new PVector(rightShoulder.x, rightShoulder.y);
        PVector rightHip2D_XY      = new PVector(rightHip.x, rightHip.y);
        
        PVector torsoOrientation_XY = PVector.sub(rightShoulder2D_XY, rightHip2D_XY);
        PVector upperArmOrientation_XY = PVector.sub(rightElbow2D_XY, rightShoulder2D_XY); 
        
        float shoulderAngle_XY = angleOf(rightElbow2D_XY, rightShoulder2D_XY, torsoOrientation_XY);
        float elbowAngle_XY    = angleOf(rightHand2D_XY, rightElbow2D_XY, upperArmOrientation_XY);
        float rightElbow_z    = rightElbow.z;
        
        PVector differenceVector  = PVector.sub(leftHand,rightHand);
        PVector differenceVector1 = PVector.sub(rightElbow, rightShoulder);    //Right Large Arm
        
        float magnitude = differenceVector.mag();    
        float magnitude1 = differenceVector1.mag();  
        
        differenceVector.normalize();
        differenceVector1.normalize();
        
        Angle_Shoulder_Out     = PApplet.parseInt(shoulderAngle_XY);
        Angle_Shoulder_Forward = PApplet.parseInt((rightShoulder.z - rightElbow.z));
        Angle_Elbow            = PApplet.parseInt(elbowAngle_XY);
        
        /*------------Servo angle-----------*/
        a = PApplet.parseInt(map(Angle_Shoulder_Forward,-150,magnitude1,70,5));
        if(a>70) a=70; if(a<5) a=5;
        
        b = PApplet.parseInt(map(Angle_Shoulder_Out,20,55,140,180)); 
        if(b > 180) b = 180; if(b < 140) b = 140;
        
        c = PApplet.parseInt(map(Angle_Elbow,100,180,30,75)); 
        if(c > 75) c = 75; if(c < 30) c = 30;
        
        if(c<45)
          d = 1;
        if(c>45)
          d = 0;
          
        /*------------TEXT------------*/ 
        fill(255, 0, 0);
        scale(1);
        text("Angle_Elbow: " + Angle_Elbow + "\n" + "Angle_Shoulder_Out: " + Angle_Shoulder_Out + "\n" + "Angle_Shoulder_Forward: " + Angle_Shoulder_Forward + "\n" + "Big Arm Length:"  +  PApplet.parseInt(magnitude1) + "\n" + "a:"  + a  + "\n" + "b:"  + b, 0, 500);
        text("c:" + c,820,500);
        
        /*------------TEXT------------*/ 
        
        //fill(255,0,0);
        //stroke(#ADFF2F);
        //kinect.drawLimb(userID, SimpleOpenNI.SKEL_LEFT_HAND, SimpleOpenNI.SKEL_RIGHT_HAND);
        //stroke(#FFFF00);
        //kinect.drawLimb(userID, SimpleOpenNI.SKEL_LEFT_HAND, SimpleOpenNI.SKEL_LEFT_SHOULDER);
        //pushMatrix();
          //fill(abs(differenceVector.x) * 255, abs(differenceVector.y) * 255, abs(differenceVector.z) * 255);
          //text("m:" + magnitude, 10 , 50);
          //text("m:" + magnitude1, 10 , 80);
        //popMatrix();
      }
    }
    send_data();
  }
  
  if(start_flag == 2)
  {
    
  }
}
/*---------------------------------------------------------------------------------------------------------*/
public void drawSkeleton(int userID)
{
  // get 3D position of head
  kinect.getJointPositionSkeleton(userID, SimpleOpenNI.SKEL_HEAD, headPosition);
  kinect.getJointPositionSkeleton(userID, SimpleOpenNI.SKEL_HEAD, headPosition);
  // convert real world point to projective space
  kinect.convertRealWorldToProjective(headPosition, headPosition);
  // create a distance scalar related to the depth in z dimension
  distanceScalar = (525/headPosition.z);
  // draw the circle at the position of the head with the head size scaled by the distance scalar
  ellipse(headPosition.x, headPosition.y, distanceScalar*headSize, distanceScalar*headSize);

  strokeWeight(6); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP);  
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE); 
  kinect.drawLimb(userID, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT); 
  //Draw the body joint//
  noStroke();
  fill(0,255,0);//GREEN
  drawJoint(userID, SimpleOpenNI.SKEL_HEAD); 
  drawJoint(userID, SimpleOpenNI.SKEL_NECK); 
  drawJoint(userID, SimpleOpenNI.SKEL_LEFT_SHOULDER); 
  drawJoint(userID, SimpleOpenNI.SKEL_LEFT_ELBOW); 
  drawJoint(userID, SimpleOpenNI.SKEL_NECK); 
  drawJoint(userID, SimpleOpenNI.SKEL_RIGHT_SHOULDER); 
  drawJoint(userID, SimpleOpenNI.SKEL_RIGHT_ELBOW); 
  drawJoint(userID, SimpleOpenNI.SKEL_TORSO); 
  drawJoint(userID, SimpleOpenNI.SKEL_LEFT_HIP); 
  drawJoint(userID, SimpleOpenNI.SKEL_LEFT_KNEE); 
  drawJoint(userID, SimpleOpenNI.SKEL_RIGHT_HIP); 
  drawJoint(userID, SimpleOpenNI.SKEL_LEFT_FOOT); 
  drawJoint(userID, SimpleOpenNI.SKEL_RIGHT_KNEE); 
  drawJoint(userID, SimpleOpenNI.SKEL_LEFT_HIP); 
  drawJoint(userID, SimpleOpenNI.SKEL_RIGHT_FOOT); 
  drawJoint(userID, SimpleOpenNI.SKEL_RIGHT_HAND); 
  drawJoint(userID, SimpleOpenNI.SKEL_LEFT_HAND); 
  fill(0, 0, 255);//BLUE
  drawJoint(userID, SimpleOpenNI.SKEL_LEFT_FINGERTIP);
  drawJoint(userID, SimpleOpenNI.SKEL_RIGHT_FINGERTIP);
}
/*---------------------------------------------------------------------------------------------------------*/
public void drawJoint(int userID, int jointID)
{
  PVector joint = new PVector();
  float confidence = kinect.getJointPositionSkeleton(userID, jointID, joint);
  if(confidence < 0.5f)
  {
    return;
  }
  PVector convertedJoint = new PVector();
  kinect.convertRealWorldToProjective(joint, convertedJoint);
  ellipse(convertedJoint.x, convertedJoint.y, 10, 10);
}
/*---------------------------------------------------------------------------------------------------------*/
public void onNewUser(SimpleOpenNI kinect, int userID)
{
  println("start pose detection");   
  kinect.startTrackingSkeleton(userID);           // start tracking of user id
}
/*---------------------------------------------------------------------------------------------------------*/
public float angleOf(PVector one, PVector two, PVector axis)
{
  PVector limb = PVector.sub(two, one);
  return degrees(PVector.angleBetween(limb, axis));
}
/*---------------------------------------------------------------------------------------------------------*/
public void send_data()
{
  out[0] = PApplet.parseByte(a);
  out[1] = PApplet.parseByte(b);
  out[2] = PApplet.parseByte(c);
  out[3] = PApplet.parseByte(d);
  port.write(out);
  delay(8);
}
/*---------------------------------------------------------------------------------------------------------*/
public void Display_Init()
{
  image(BackGround,0,0,1280,680);
}
/*---------------------------------------------------------------------------------------------------------*/
public void START(int theValue) 
{
  start_flag = theValue;
}

public void Introduction(int theValue) 
{
  start_flag = theValue;
}

public void Back(int theValue) 
{
  start_flag = theValue;
}
/*---------------------------------------------------------------------------------------------------------*/
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Kinect_Artificial_Limb" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
