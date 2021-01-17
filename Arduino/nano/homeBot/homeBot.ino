

/*
  This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
  It won't work with v1.x motor shields! Only for the v2's with built in PWM
  control

  For use with the Adafruit Motor Shield v2
  ---->	http://www.adafruit.com/products/1438
*/

#define LEDR 22
#define LEDG 23
#define LEDB 24

#include <Adafruit_MotorShield.h>
#include <MadgwickAHRS.h>
#include <Arduino_LSM9DS1.h>
#include <string>
#include <iostream>
#include "homebot.h"

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h> 
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <robot_setup_tf/homebotToPi.h>

//void encoderSetup();
int analogPin = A0; // battery voltage thru a voltage divider to ensure its below 3.3v
unsigned long microsPerReading, microsPrevious;
Madgwick filter;

ros::NodeHandle  nh;
//nav_msgs::Odometry od;
//geometry_msgs::TransformStamped t;
robot_setup_tf::homebotToPi homebot_msg;
std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//std_msgs::Float32 headingDelta_msg;
std_msgs::Float32 inhibit_msg;
//ros::Publisher headingDelta_pub("headingDelta", &headingDelta_msg);
//ros::Publisher Odometry("/nav_msgs/Odometry", &od);
ros::Publisher homebot_pub("homebot_data",&homebot_msg);
//tf::TransformBroadcaster broadcaster;

void messageCb( const geometry_msgs::Twist& cmd_msg);
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb );
void inhibitCb( const std_msgs::Float32& inhibit_msg);
ros::Subscriber<std_msgs::Float32> sub_inhibit("inhibit",inhibitCb );

float loopFreq=1000000/loopRateMicros;  //hz
bool motorEnable=false;
int staticSpeed=75;
int manualSpeed[2] = {0,0};
int autoSpeed[2]={0,0};
int deltaMotor=0;


float yaw;
float yawDelta;
float yawPoints[4]={47.42,143.76,243.91,329.19};
float initialYaw;
float headingTarget=0.0;

float mx,my,mz; //units = Deg/second
float gx,gy,gz; //degrees/sec
float magYaw;
float magXOffset=31.65;//32.42;//3.68;//17.2;//85.91;
float magYOffset=9.29;//5.37;//2.69;//18.74;//50.02;
float gXOffset=.53;//4;//.56;
float gYOffset=.65;//.91;//.70;//1.11;
float gZOffset=.62;//.62;//.44;
float motorSpeed[2];
int portSpeed=0;
volatile int encoderCount[2] = {0, 0}; //port=0 starboard=1
float tachDelta[2]={0,0};
float position[2]={0,0};
static float joyStickYAnalog=0;
  static float joyStickXAnalog=0;
  static float joyStickInhibitAnalog=0;
  static boolean joyStickXWasOn=false;
  static boolean joyStickYWasOn=false; 
  static boolean buttonXOn=false;
  static boolean buttonXWasOff=false;
  static boolean buttonYOn=false;
  static boolean buttonYWasOff=false;
  static boolean buttonAOn=false;
  static boolean buttonAWasOff=false;
  static boolean buttonBOn=false;
  static boolean buttonBWasOff=false;
  static unsigned long inhibitWatchdogTimer=millis();
//volatile int encoderCountStarboard=0;
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *portMotor = AFMS.getMotor(4);
Adafruit_DCMotor *starboardMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

//=============================================================================================
void setup() {
  if(!rosOn) Serial.begin(115200);           // set up Serial library at 9600 bps
  if(!rosOn) Serial.setTimeout(1000);   
  if(!rosOn) Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  if(rosOn){
    nh.initNode();
    //nh.advertise(chatter);
    nh.subscribe(sub_vel);
    nh.subscribe(sub_inhibit);
    //broadcaster.init(nh);   //for TF broadcast 
    //nh.advertise(headingDelta_pub); 
    nh.advertise(homebot_pub);   
  }   
  
  encoderSetup();
  if (!IMU.begin()) {
    if(!rosOn) Serial.println("Failed to initialize IMU!");
    while (1);
  }
  AFMS.begin(1000);  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  portMotor->setSpeed(manualSpeed[0]);
  starboardMotor->setSpeed(manualSpeed[1]);
  //myMotor->run(FORWARD);
  // turn on moter
  portMotor->run(RELEASE);
  starboardMotor->run(RELEASE);
  filter.begin(loopFreq*.9);  //50hz, 10hz
  // initialize variables to pace updates to correct rate
  //microsPerReading = 1000000 / 100;
  // microsPrevious = micros();
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(LEDR,OUTPUT);
  pinMode(LEDG,OUTPUT);
  pinMode(LEDB,OUTPUT);
  digitalWrite(LEDR,true);
  digitalWrite(LEDG,true);
  digitalWrite(LEDB,true);
  bool ledState=true;
  
  for (int i = 0; i < 600; i++) {
    delay(20);
    digitalWrite(LED_BUILTIN,ledState);
    digitalWrite(LEDR,i%3!=0);
    digitalWrite(LEDG,(i+1)%3!=0);
    digitalWrite(LEDB,(i+2)%3!=0);
    ledState=!ledState;    
    yaw=updateYaw();
    if(yaw>-1) initialYaw=yaw;
  }
  digitalWrite(LEDR,true);
  digitalWrite(LEDG,true);
  digitalWrite(LEDB,true);
}
//==================================================================
void inhibitCb( const std_msgs::Float32& inhibit_msg){
  if (inhibit_msg.data!=-1){
      motorEnable=false;
      portMotor->run(RELEASE);
      starboardMotor->run(RELEASE);         
    }
    else    
    {
      motorEnable=true;
      inhibitWatchdogTimer=millis();
      } 
    /*
    if (inhibitWatchdogTimer+300<millis()){  //in case of fail to comunicate to joystick inhibit
      motorEnable=false;
      portMotor->run(RELEASE);
      starboardMotor->run(RELEASE);  
    }
    */ 
  }

//==================================================================
void messageCb( const geometry_msgs::Twist& cmd_msg) {
  
  //if ( cmd_msg.angular.z == 0 && cmd_msg.linear.x == 0 ) {
    
    joyStickYAnalog=cmd_msg.linear.x;
    joyStickXAnalog=-cmd_msg.angular.z;
    
  
}
//============================================================================================
void calcOdometry(float yaw, float oldYaw){
  static float integratedEncoderDelta=0;
  float tachDeltaSum=(tachDelta[0]+tachDelta[1])/2;
  float tachDeltaDelta=(tachDelta[0]-tachDelta[1])/2;
  yawDelta=yaw-oldYaw;  
  if(yawDelta>180) yawDelta-=180;
  if(yawDelta<-180) yawDelta+=180;
  float avgYaw=oldYaw+yawDelta/2;
  if(avgYaw>360) avgYaw-=360;
  if(avgYaw<0) avgYaw+=360;
  float deltaPos[2];
  deltaPos[0]=-mPerTach*tachDeltaSum*cos(avgYaw*M_PI/180);  //changes sign to make odom match map
  deltaPos[1]=mPerTach*tachDeltaSum*sin(avgYaw*M_PI/180);
  position[0]+=deltaPos[0]; // 0 is x direction based on 0 degrees is +x
  position[1]+=deltaPos[1];
  //od.pose.pose.position.x = position[0];//sumX*metersPerTach; // 1.016e-4m/tach pulse    
  //od.pose.pose.position.y = position[1];//sumY*metersPerTach;
  //od.twist.twist.linear.x = deltaPos[0];//(intY)*metersPerTach/(loopTimeTarget*.001);   //m/s
  //od.twist.twist.linear.y = deltaPos[1];//(intX)*metersPerTach/(loopTimeTarget*.001);   //m/s 
  //headingDelta_msg.data=yaw;
  homebot_msg.arduino_micros=micros();
  homebot_msg.integrated_X=position[0];  //m
  homebot_msg.integrated_Y=position[1];
  homebot_msg.X_rate=mPerTach*(motorSpeed[0]+motorSpeed[1])/2*cos(avgYaw*M_PI/180);  //m/s
  homebot_msg.Y_rate=mPerTach*(motorSpeed[0]+motorSpeed[1])/2*sin(avgYaw*M_PI/180);
  homebot_msg.yaw_IMU=yaw*M_PI/180; //rad  
  homebot_msg.yaw_rate_IMU=yawDelta*M_PI/180*1E6/loopRateMicros;    //deg/sample*rad/deg*(micros/sec)/(micros/sample)=r/s
  homebot_msg.yaw_rate_encoders=mPerTach*(motorSpeed[0]-motorSpeed[1])/mBetweenTreads;  //M/tach * tach/s /M = rad/sec
  integratedEncoderDelta+=tachDeltaDelta*mPerTach;  //tach*tach/M=M
  while (integratedEncoderDelta>M_PI*mBetweenTreads){  //sort of ramainder above 2* PI * R= PI * R*2  
    integratedEncoderDelta-=M_PI*mBetweenTreads;  
  }
  while (integratedEncoderDelta<0){
    integratedEncoderDelta+=M_PI*mBetweenTreads;  
  }
  homebot_msg.yaw_encoders=integratedEncoderDelta/(mBetweenTreads/2);  //M/M=rad
  homebot_pub.publish(&homebot_msg);
  if(!rosOn){
     Serial.print("odometry,");
     Serial.print(millis());     //in millis 
     Serial.print("  ");
     Serial.print(position[0],6);  //in encoder counts
     Serial.print("  ");
     Serial.print(position[1],6);
     Serial.print("  ");
     Serial.print(yaw*M_PI/180,3);          //rad
     Serial.print("  ");
     Serial.print(deltaPos[0]/loopRateMicros*1.0e6f,6);  //im meters suposed to be in M/sec
     Serial.print("  ");
     Serial.print(deltaPos[1]/loopRateMicros*1.0e6f,6);
     Serial.print("  ");
     Serial.println(yawDelta*M_PI/180/(loopRateMicros/1.0e6),6);   //suposed to be rad/sec   
  }
}
//============================================================================================
float yaw2CorrectedYaw (float yaw){
  float correctedYaw;
  float internalPoints[4];
  float correctedPoint;
  float correctionWithinZone;
  int iP1;
  for(int i=0;i<4;i++){
    internalPoints[i]=yawPoints[i]-yawPoints[0];
  }
  for (int i=0;i<4;i++){
   if (i==3){
    iP1=0;
    internalPoints[0]=360;
   }  
   else 
    iP1=i+1;     
     
   if(yaw<internalPoints[iP1]){
    correctedPoint=90*i;
    correctionWithinZone=90/(internalPoints[iP1]-internalPoints[i])*(yaw-internalPoints[i]);
    correctedYaw=correctedPoint+correctionWithinZone;
    /*
    if(!rosOn) Serial.print("correction for zone=");
    if(!rosOn) Serial.print(correctedPoint);
    if(!rosOn) Serial.print(" ,fine correction=");
    if(!rosOn) Serial.println(correctionWithinZone);
    if(!rosOn) Serial.print("yaw,corrected yaw=");
    if(!rosOn) Serial.print(yaw);
    if(!rosOn) Serial.print(" , ");
    if(!rosOn) Serial.println(correctedYaw);
    */
    return correctedYaw;
   }
  }
  return correctedYaw=9999;
}
//============================================================================================
void processJoyStick(String inString) {
  int nextCommaLoc=0; 
  int lastCommaLoc=0;    
      lastCommaLoc=nextCommaLoc;
      nextCommaLoc=inString.indexOf(",",lastCommaLoc);  
      if (inString.charAt(nextCommaLoc-3)=='1') buttonXOn=true;
      else {
        buttonXOn=false;
        buttonXWasOff=true;
      }
      lastCommaLoc=nextCommaLoc;
      nextCommaLoc=inString.indexOf(",",lastCommaLoc+1);  
      if (inString.charAt(nextCommaLoc-3)=='1') buttonYOn=true;
      else {
        buttonYWasOff=true;
        buttonYOn=false;
      }
      lastCommaLoc=nextCommaLoc;
      nextCommaLoc=inString.indexOf(",",lastCommaLoc+1);  
      if (inString.charAt(nextCommaLoc-3)=='1') buttonAOn=true;
      else {
        buttonAWasOff=true;
        buttonAOn=false;
      }
      lastCommaLoc=nextCommaLoc;
      nextCommaLoc=inString.indexOf(",",lastCommaLoc+1);  
      if (inString.charAt(nextCommaLoc-3)=='1') buttonBOn=true;
      else { 
        buttonBOn=false;
        buttonBWasOff=true;
      }
      lastCommaLoc=nextCommaLoc; 
      nextCommaLoc=inString.indexOf(",",lastCommaLoc+1); 
      joyStickXAnalog=inString.substring(lastCommaLoc+1,nextCommaLoc).toFloat();
      lastCommaLoc=nextCommaLoc; 
      nextCommaLoc=inString.indexOf(",",lastCommaLoc+1); 
      joyStickYAnalog=inString.substring(lastCommaLoc+1,nextCommaLoc).toFloat();
      lastCommaLoc=nextCommaLoc; 
      nextCommaLoc=inString.indexOf(",",lastCommaLoc+1); 
      joyStickInhibitAnalog=inString.substring(lastCommaLoc+1,nextCommaLoc).toFloat();
      inhibitWatchdogTimer=millis();
      if (joyStickYAnalog>.1){        //turn the opsite way during backup
      joyStickXAnalog=-joyStickXAnalog;
      }
}

//=============================================================================================
void clearSerialBuffer(){
  if(!rosOn){ 
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
 }    
} 
//=============================================================================================
void myDelay(unsigned long iDelay){
 unsigned long startMillis=millis();
 while(millis()<startMillis+iDelay){
  clearSerialBuffer(); 
 }  
} 
//==========================================================================================
float readAnalogVoltage(){
  int rawReading;
  float batVoltage;
  rawReading = analogRead(analogPin);  // read the input pin
  batVoltage=rawReading*18.2/1024;
  //Serial.print("battery Voltage=");
  //Serial.println(batVoltage);          // debug value
  return batVoltage;
}
//==========================================================================================
void calibrate(){
  float calX[4]={0,0,0,0}; // 0=360deg, 1=90deg, 2=180deg, 3=270deg
  float calY[4]={0,0,0,0};
  float internalYaw;
  
  float calXRange[2]={9999,-9999};// 0=min, 1=max  
  float calYRange[2]={9999,-9999};
  unsigned int startCountMillis=millis();
  unsigned int timerOffset=0;
  //float mx,my,mz;
  float cumMx, cumMy;
  float cumGX,cumGY,cumGZ;
  int   measurements;
  startCountMillis=millis();
  digitalWrite(LED_BUILTIN,true);
  if(!rosOn) Serial.println("Calibration in progress, do not move bot until instructed");
  measurements=0;
  cumMx=0;
  cumMy=0;
  cumGX=0;
  cumGY=0;
  cumGZ=0;
  timerOffset+=2000;
  
  if(!rosOn) Serial.print("current millis=");
  if(!rosOn) Serial.println(millis());
  if(!rosOn) Serial.print("Target millis=");
  if(!rosOn) Serial.println(startCountMillis+timerOffset);
  myDelay(2000); 
  //while(millis()<(startCountMillis+timerOffset)){
  //Serial.print("current millis=");
  //Serial.println(millis());  
  //}    //wait to stabalize
  timerOffset+=5000;
  if(!rosOn) Serial.println("Starting measurements now");
  while(millis()<startCountMillis+timerOffset){ //measure 0 heading
   clearSerialBuffer(); 
   yaw=updateYaw(); 
   myDelay(loopRateMicros/1000);
   if (yaw>-1.0) {     
     cumMx+=mx;     
     cumMy+=my;
     cumGX+=gx;
     cumGY+=gy;
     cumGZ+=gz;
     
     measurements++;
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my;
     yawPoints[0]=yaw; 
   }     
  }  
  gXOffset=cumGX/measurements;
  gYOffset=cumGY/measurements;
  gZOffset=cumGZ/measurements;
  calX[0]=cumMx/measurements;
  calY[0]=cumMy/measurements;
  measurements=0;
  cumMx=0;
  cumMy=0;
  digitalWrite(LED_BUILTIN,false);
  if(!rosOn) Serial.println("Please turn bot right to 90 deg and wait");
  timerOffset+=3000;
  while(millis()<startCountMillis+timerOffset){   
   clearSerialBuffer();
   yaw=updateYaw(); 
   myDelay(loopRateMicros/1000);
   if (yaw>-1.0) {    
     if(mx>calXRange[1]) calXRange[1]=mx;  //contineu to search for max/min magnatometer during turn
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
   }
   
  }
  digitalWrite(LED_BUILTIN,true);
  if(!rosOn) Serial.println("90 degree Measurement in progress do not move bot until instructed");
  timerOffset+=5000;
  while(millis()<startCountMillis+timerOffset){ //measure 90 heading
   clearSerialBuffer();  
   yaw=updateYaw();
   myDelay(loopRateMicros/1000); 
   if (yaw>-1.0) {
     cumMx+=mx;
     cumMy+=my;
     measurements++;
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
     yawPoints[1]=yaw;
   }     
  } 
  calX[1]=cumMx/measurements;
  calY[1]=cumMy/measurements;
  measurements=0;
  cumMx=0;
  cumMy=0;
  digitalWrite(LED_BUILTIN,false);
  if(!rosOn) Serial.println("Please turn bot right to 180 deg and wait");
  timerOffset+=3000;
  while(millis()<startCountMillis+timerOffset){
   clearSerialBuffer();
   yaw=updateYaw();
   myDelay(loopRateMicros/1000); 
   if (yaw>-1.0) {     
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
   }  
  } 
  if(!rosOn) Serial.println("180 degree Measurement in progress do not move bot until instructed");
  digitalWrite(LED_BUILTIN,true);
  timerOffset+=5000;
  while(millis()<startCountMillis+timerOffset){ //measure 180 heading
   clearSerialBuffer();
   yaw=updateYaw();
   myDelay(loopRateMicros/1000); 
   if (yaw>-1.0) {
     cumMx+=mx;
     cumMy+=my;
     measurements++;
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
     yawPoints[2]=yaw;   
   }     
  }
  calX[2]=cumMx/measurements;
  calY[2]=cumMy/measurements;
  measurements=0;
  cumMx=0;
  cumMy=0;
  digitalWrite(LED_BUILTIN,false);
  if(!rosOn) Serial.println("Please turn bot right to 270 deg and wait");
  timerOffset+=3000;
  while(millis()<startCountMillis+timerOffset){
  clearSerialBuffer();
  yaw=updateYaw();
  myDelay(loopRateMicros/1000); 
   if (yaw>-1.0) {     
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
   }  
  }
  if(!rosOn) Serial.println("270 degree Measurement in progress do not move bot until instructed");
  digitalWrite(LED_BUILTIN,true);
  timerOffset+=5000;
  while(millis()<startCountMillis+timerOffset){  //measure 270 heading
  clearSerialBuffer();
  yaw=updateYaw(); 
  myDelay(loopRateMicros/1000);
   if (yaw>-1.0) {
     cumMx+=mx;
     cumMy+=my;
     measurements++;
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my;     
     yawPoints[3]=yaw; 
   }  
  }
  digitalWrite(LED_BUILTIN,false);
  if(!rosOn) Serial.println("Please turn bot right to 360 deg and wait");
  timerOffset+=5000;
  while(millis()<startCountMillis+timerOffset){
  clearSerialBuffer();
  yaw=updateYaw();
  myDelay(loopRateMicros/1000); 
   if (yaw>-1.0) {     
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
     initialYaw=0;
   }  
  }  
  calX[3]=cumMx/measurements;
  calY[3]=cumMy/measurements;
  measurements=0;
  cumMx=0;
  cumMy=0;
  if(!rosOn) Serial.println("Final Measurement in progress do not move bot until instructed");
  digitalWrite(LED_BUILTIN,true);
  timerOffset+=3000;
  while(millis()<startCountMillis+timerOffset){  //measure initial yaw 
    clearSerialBuffer();
    yaw=updateYaw();
    myDelay(loopRateMicros/1000);
    if (yaw>-1.0) initialYaw=yaw;
  }
  //Serial.print("Original Mag offset (x,Y) =");
  //Serial.print(magXOffset);
  //Serial.print(", ");
  //Serial.println(magYOffset);
  //Serial.print("Yaw at 0 deg, 90deg, 180deg, 270deg =");
  //Serial.print(yawPoints[0]);
  //Serial.print(", ");
  //Serial.print(yawPoints[1]);
  //Serial.print(", ");
  //Serial.print(yawPoints[2]);
  //Serial.print(", ");
  //Serial.println(yawPoints[3]); 
  magXOffset=(calXRange[0]+calXRange[1])/2;
  magYOffset=(calYRange[0]+calYRange[1])/2;
  digitalWrite(LED_BUILTIN,false);
  if(!rosOn) Serial.println("Calibration complete Thank You!");
  if(!rosOn) Serial.print("recalibrated Mag offset (x,Y) Gz =");
  if(!rosOn) Serial.print(magXOffset);
  if(!rosOn) Serial.print(", ");
  if(!rosOn) Serial.println(magYOffset);
  if(!rosOn) Serial.print("recalibrated G offset (x,y,z)");
  if(!rosOn) Serial.print(gXOffset);
  if(!rosOn) Serial.print(", ");
  if(!rosOn) Serial.print(gYOffset); 
  if(!rosOn) Serial.print(", ");  
  if(!rosOn) Serial.println(gZOffset);  
  //Serial.print("max X, min X, max y, min Y =");
  //Serial.print(calXRange[1]);
  //Serial.print(", ");
  //Serial.print(calXRange[0]);
  //Serial.print(", ");
  //Serial.print(calYRange[1]);
  //Serial.print(", ");
  //Serial.println(calYRange[0]);
  //Serial.print("X@ 0, 90, 180, 270 = ");
  //Serial.print(calX[0]);
  //Serial.print(", ");
  //Serial.print(calX[1]);
  //Serial.print(", ");
  //Serial.print(calX[2]);
  //Serial.print(", ");
  //Serial.println(calX[3]);
  //Serial.print("Y@ 0, 90, 180, 270 = ");
  //Serial.print(calY[0]);
  //Serial.print(", ");
  //Serial.print(calY[1]);
  //Serial.print(", ");
  //Serial.print(calY[2]);
  //Serial.print(", ");
  //Serial.println(calY[3]);  
  /*Serial.println("do you want to save these values? Y or N");
  String newString=Serial.readString();
  if (newString="y") {
  */
    if(!rosOn) Serial.print("OK, save calibration_data,");
    if(!rosOn) Serial.print(magXOffset);
    if(!rosOn) Serial.print(",");
    if(!rosOn) Serial.print(magYOffset);
    if(!rosOn) Serial.print(",");
    if(!rosOn) Serial.print(gXOffset);
    if(!rosOn) Serial.print(",");
    if(!rosOn) Serial.print(gYOffset);
    if(!rosOn) Serial.print(",");
    if(!rosOn) Serial.print(gZOffset);
    if(!rosOn) Serial.print(",");
    if(!rosOn) Serial.print(yawPoints[0]);
    if(!rosOn) Serial.print(",");
    if(!rosOn) Serial.print(yawPoints[1]);
    if(!rosOn) Serial.print(",");
    if(!rosOn) Serial.print(yawPoints[2]);
    if(!rosOn) Serial.print(",");
    if(!rosOn) Serial.println(yawPoints[3]);
    
  
  
}
//===========================================================================================
int motDeltaFromYaw(float yaw, float HeadingTarget, float yawSpeedTargetMagnitude) {
  //float yawSpeedTargetMagnitude; 
  float yawSpeedTarget;
  float yawDeadBand=.5;
  float yawSPGain=.005; 
  float yawErr=HeadingTarget-yaw;
  static float yawOld=0;             //only initialize first time
  float motorSpeedGain=.01;
  float tachTurnSpeedMeasured=(motorSpeed[0]-motorSpeed[1])/2*motorSpeedGain;
  float yawDelta=tachTurnSpeedMeasured;//yaw-yawOld;  
  
  yawOld=yaw;
  if (yawErr>180) yawErr=yawErr-360;
  if (yawErr<-180) yawErr=yawErr+360;
  if (yawErr>90) yawErr=90;
  if (yawErr<-90) yawErr=-90;

  if (abs(yawSpeedTargetMagnitude)==0){
   if (abs(yawErr)>5) yawSpeedTargetMagnitude=yawSpeedTargetMagnitudeDefault;
   else yawSpeedTargetMagnitude=yawSpeedTargetMagnitudeDefault*.20;
  }
  else yawSpeedTargetMagnitude=abs(yawSpeedTargetMagnitude);
  

  //dynamic vs Static section
  if(abs(motorSpeed[0]*motorSpeedGain)>100 && abs(motorSpeed[1]*motorSpeedGain)>100 && (motorSpeed[1]*motorSpeed[0])>=0){  //Moving straight= low gain and no deadband
  
    yawSpeedTargetMagnitude=yawSpeedTargetMagnitude*.2;
    yawDeadBand=.5;
  }
  else{
    if(abs(motorSpeed[0]*motorSpeedGain)>100 && abs(motorSpeed[1]*motorSpeedGain)>100){
      yawSpeedTargetMagnitude=yawSpeedTargetMagnitude*.6;  //dynamic lower gain even if turning
    }
  }
  //
  if (abs(yawErr)<=yawDeadBand) return 0;
  yawSpeedTarget=0;
  if (yawErr<-yawDeadBand) yawSpeedTarget=-yawSpeedTargetMagnitude;
  if (yawErr>=yawDeadBand) yawSpeedTarget=yawSpeedTargetMagnitude;
  float yawDeltaErr=yawSpeedTarget*50000-yawDelta;
  //if (yawDeltaErr>180) yawDeltaErr=yawDeltaErr-360;
  int motorDeltaOut=round(yawDeltaErr*yawSPGain);  
  //if (yawDeltaErr<-180) yawDeltaErr=yawDeltaErr+360;
  //Serial.print("yaw rate =");
  //Serial.print(yawDelta);
  motorDeltaOut=max(motorDeltaOut,-254);
  motorDeltaOut=min(motorDeltaOut,254);
  return motorDeltaOut; 
 
}
//==========================================================================================
float updateYaw() {
  static float ax, ay, az;    //in Gs 
  //static float gx, gy, gz;    //in Deg/second
  //static float mx, my, mz;
  //static unsigned long microsPrevious=micros();
  //unsigned long microsNow=micros();
  //unsigned long microsPerReading=loopRateMicros;  //10000;  // 100hz
  float gGain=1.0;
  
  float internalYaw;
  // check if it's time to read data and update the filter
  //if (microsNow >= microsPrevious+microsPerReading)
  {
  //  microsPrevious = microsNow;

    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);  //avail rate 104hz
    }
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
    }
    if (IMU.magneticFieldAvailable()) {   //avail rate= 20hz
      IMU.readMagneticField(mx, my, mz);
    }
    //Serial.print(mx);
    //Serial.print(" , ");
    //Serial.print(my); 
    //Serial.print(" , ");
    //Serial.println(mz);
    magYaw=atan2(-(my-magYOffset),(mx-magXOffset))*180/M_PI+180;

    filter.update((gx-gXOffset)*gGain, (-gy+gYOffset)*gGain, (gz-gZOffset)*gGain, ax, -ay, az, -(my - magYOffset), (mx - magXOffset), mz); //manualy measured offsets for x and y magnatome
    //signs of updated based on documentation. All axis now right hand rule with Z up, x,y,z.

    //double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    //double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    internalYaw = 360 - filter.getYaw();
    return internalYaw;
  }
  return -1000.0;
}


//======================================================================================
int speedControl(float targetSpeed, int tach0Input, int tach1Input)
// parm 3 and 4 is the direction of motor X tach, should be -1, 0 or 1.
// liniar speed control parm 3 and 4 = 1,1
// rotational speed control parm3 and parm 4 = 1,-1

{ 
  float speedSGain=.9;
  float speedPGain=.00;
  float speedIGain=.00;
  
  const int integratorDepth=15;
  static float integratorArray[integratorDepth][2]={ };
  float integratorSum=0;  
  int lowSpeedOffset=10; //min motor dac to get reliable movment
  float calculatedSpeed=(tach0Input*motorSpeed[0]+tach1Input*motorSpeed[1])/(abs(tach0Input)+abs(tach1Input));  
  float err=targetSpeed*13.0f-calculatedSpeed*.01; //motorSpeed[motNo]; 
  int controlType=0; //fwd servo
  if (tach0Input+tach1Input==0) controlType=1;  //turning servo
   
  static int integratorCurrentPointer=0;
  //integratorArray[i,C]=err;
  //if (i<integratorDepth) i++;
  //if (i==integratorDepth) i=0;
  //integratorSum=0;
  integratorArray[integratorCurrentPointer][controlType]=err;
  if (integratorCurrentPointer+1>=integratorDepth)integratorCurrentPointer=0;
  else integratorCurrentPointer++;
  for (int j=0;j<integratorDepth;j++) integratorSum+=integratorArray[j][controlType];  
  //Serial.println(integratorSum/float(integratorDepth)*speedIGain);
  int control=int(targetSpeed*speedSGain+err*speedPGain+integratorSum*speedIGain/float(integratorDepth));  
  if (control>0) control=control+(lowSpeedOffset-1);
  if (control<0) control=control-(lowSpeedOffset-1);
  control=max(control,-254);
  control=min(control,254);
  return control;
}
//===========================================================================================
float measureSpeed(int motNo)
{  
  float speedOut = motorSpeed[motNo];
  tachDelta[motNo]=0;
  //tachDelta[1]=0;
  unsigned long currentMicros = millis();  
  static int lastEncoderCount[2] = {encoderCount[0], encoderCount[1]};
  static unsigned long whenLastEncoderChange[2] = {millis(), millis()};  
  {
    
    if (encoderCount[motNo] != lastEncoderCount[motNo])
    {
      speedOut = 1.0e6f * float(encoderCount[motNo] - lastEncoderCount[motNo]) / float(currentMicros - whenLastEncoderChange[motNo]);//tach/sec
      tachDelta[motNo]=encoderCount[motNo] - lastEncoderCount[motNo];
      whenLastEncoderChange[motNo] = currentMicros;
      lastEncoderCount[motNo] = encoderCount[motNo];
    }
    else if (currentMicros  >= 3*loopRateMicros+whenLastEncoderChange[motNo]) {
      speedOut = 0.0f;
      whenLastEncoderChange[motNo] = currentMicros;
    }
    else {
      speedOut = speedOut / 2.0f;
      
    }

    //Serial.print("speed micros=");//port encoder count = ");
    //Serial.print(micros());//encoderCount[0]);
    //Serial.print(" ,star encoder=");
    //Serial.print(encoderCount[1]);
    //Serial.print(" , star speed=");

    //Serial.print(" , yaw = ");
    //Serial.println(lastCount);
  }

  return speedOut;
}
//=====================================================================================
void loop() {
  //uint8_t i;
  float yawTemp;
  
  static unsigned long lastClock = micros();
  static float oldYaw=0;
  static float yawDelta=0;
  int outSpeed[2]={0,0};
  static char serialBuffer[600];
  static int bufferPos=0;
  String inString="";
  char serialInByte;
  float keyboardHeadingTarget=999.0;
  bool keyboardInputReady=false;
  bool singleKeyCommandSkip=false;
  float yawSpeedTarget;
  yawSpeedTarget=0;//yawSpeedTargetMagnitudeDefault;  
  static int     positionTarget=0;
  
  if (micros() > loopRateMicros+lastClock)
  {
    lastClock = lastClock + loopRateMicros;
    //Serial.print("main loop micros=");
    //Serial.println(lastClock);
    //readAnalogVoltage();
    //byte inString[116];
    String inString="";
    String internalString="";
    oldYaw=yaw;
    yawTemp = updateYaw();
    if (yawTemp > -1.0) {    //???????
      yaw = yawTemp - initialYaw;
      if (yaw < 0.0) yaw = yaw + 360.0;
    }
    yaw=yaw2CorrectedYaw(yaw);
    for (int i = 0; i < 2; i++)
    {
      motorSpeed[i] = measureSpeed(i); 
    }
    calcOdometry(yaw, oldYaw);
    if(!rosOn){
    while (Serial.available() > 0) 
    {          
      serialInByte=char(Serial.read());
      serialBuffer[bufferPos]=serialInByte;
      bufferPos++;
      if (bufferPos>500 || serialInByte=='%') {  //serial data formated "%payload$"
        bufferPos=0;
        internalString="";
      }      
      if (serialInByte=='$' && bufferPos>1) {         
        internalString="";       
        for(int i=0;i<bufferPos-1;i++){
          internalString+=serialBuffer[i];
        } 
        //internalString+='\0'; 
        bufferPos=0;        
      }      
   }
   }
   
   if(internalString.length()>0 ){
    //Serial.print("decoded string=");
    //Serial.println(internalString);
    
    //if (internalString.charAt(0)!='X'){
    //  Serial.print("opps string should have started with X string length=");
    //  Serial.println(internalString.length());
    //}
    //else Serial.println("sometimes the first char is X good");    
    inString=internalString;

    if(45 < inString.charAt(0) && inString.charAt(0)<57){  //is a number, joystick input
      processJoyStick(inString);      
    }     
    else 
    {//keyboardInput processing            
    //  Serial.print("decoded string= keyboard input 8888888888888888888888888888888888888888888");
    //Serial.println(internalString); 
      if (inString.substring(0,5)=="start"){
         //Serial.println("Read Key = start");
         motorEnable=true;
         singleKeyCommandSkip=true;
         
      }
      if (inString.substring(0,4)=="stop"){
         //Serial.println("Read Key = start");
         motorEnable=false;
         portMotor->run(RELEASE);
         starboardMotor->run(RELEASE);
         singleKeyCommandSkip=true;
      }
      if (inString.substring(0,5)=="calib"){
         //Serial.println("Read Key = start");
         //motorEnable=false;
         calibrate();
         singleKeyCommandSkip=true;
      }
      
        
      // = Serial.read();
      // do something different depending on the character received.
      // The switch statement expects single number values for each case;
      // in this exmaple, though, you're using single quotes to tell
      // the controller to get the ASCII value for the character.  For
      // example 'a' = 97, 'b' = 98, and so forth:
      //Serial.println(inByte);
      int inByte=serialBuffer[0];
      if( !singleKeyCommandSkip){
       switch (inByte) {

        case 'w': //forward
          manualSpeed[0]=staticSpeed;
          manualSpeed[1]=staticSpeed;          
          break;
        case 's': //stop          
          manualSpeed[0]=0;
          manualSpeed[1]=0;
          headingTarget=yaw;
          break;
        case 'a': //left
          keyboardHeadingTarget=headingTarget-5.0;
          if(keyboardHeadingTarget<0.0) keyboardHeadingTarget+=360.0; 
          //manualSpeed[0]=-staticSpeed;
          //manualSpeed[1]=staticSpeed;          
          break;
        case 'd': //right
          keyboardHeadingTarget=headingTarget+5.0;
          if(keyboardHeadingTarget>=360.0) keyboardHeadingTarget-=360.0;
          //manualSpeed[0]=staticSpeed;
          //manualSpeed[1]=-staticSpeed;
           break;
        case 'x': //backup
          manualSpeed[0]=-staticSpeed;
          manualSpeed[1]=-staticSpeed;          
          break;
        case '+': //increase speed;
          if(!rosOn) Serial.println("current speed = ");
          if(!rosOn) Serial.println(staticSpeed);          
          if (staticSpeed < 250) staticSpeed += 5;          
          if(!rosOn) Serial.println("new speed = ");
          if(!rosOn) Serial.println(staticSpeed);
          manualSpeed[0]=manualSpeed[0]/abs(manualSpeed[0])*staticSpeed;
          manualSpeed[1]=manualSpeed[1]/abs(manualSpeed[1])*staticSpeed;
          break;
        case '-': //decrease speed;
          if(!rosOn) Serial.println("current speed = ");
          if(!rosOn) Serial.println(staticSpeed);          
          if (staticSpeed > 5) staticSpeed -= 5;          
          if(!rosOn) Serial.println("new speed = ");
          if(!rosOn) Serial.println(staticSpeed);
          manualSpeed[0]=manualSpeed[0]/abs(manualSpeed[0])*staticSpeed;
          manualSpeed[1]=manualSpeed[1]/abs(manualSpeed[1])*staticSpeed;
          break;
        }  
       }
      } 
      bufferPos=0; 
      inString="";   
     }
     
    //if(abs(joyStickYAnalog)<.01) joyStickYAnalog=0.0;      
    //if(abs(joyStickXAnalog)<.01) joyStickXAnalog=0.0;      
    
    
    if (buttonXOn && buttonXWasOff){
      buttonXWasOff=false;
      keyboardHeadingTarget=headingTarget-90.0;
      if(keyboardHeadingTarget<0.0) keyboardHeadingTarget+=360.0;
      headingTarget=keyboardHeadingTarget;
      //yawSpeedTarget=100;
    }
    if (buttonBOn && buttonBWasOff){
      buttonBWasOff=false;
      keyboardHeadingTarget=headingTarget+90.0;
      if(keyboardHeadingTarget>359) keyboardHeadingTarget-=360.0;
      headingTarget=keyboardHeadingTarget;
      //yawSpeedTarget=100;    
    }
    if(abs(joyStickYAnalog)>.1){    
      manualSpeed[1]=-joyStickYAnalog*255;
      joyStickYWasOn=true;
    }
    
    if(abs(joyStickYAnalog)<.1 && joyStickYWasOn){
      manualSpeed[1]=0;
      headingTarget=yaw;
      joyStickYWasOn=false;  
    }
    autoSpeed[1]=speedControl(manualSpeed[1],1,1);
    
    if(joyStickXAnalog>=.1){
      headingTarget=yaw+10;
      joyStickXWasOn=true;
      if (headingTarget>359) headingTarget-=360;
      yawSpeedTarget=joyStickXAnalog*2.5;
    }
    if(joyStickXAnalog<=-.1){
      headingTarget=yaw-10;
      joyStickXWasOn=true;
      if (headingTarget<0) headingTarget+=360;
      yawSpeedTarget=-joyStickXAnalog*2.5;
    }
    if(abs(joyStickXAnalog)<.1){ 
      if(joyStickXWasOn){  
        joyStickXWasOn=false;
        headingTarget=yaw;
      }  
      else if(keyboardHeadingTarget<999) headingTarget=keyboardHeadingTarget; 
    } 
    
    //deltaMotor=speedControl(motDeltaFromYaw(yaw,headingTarget,yawSpeedTarget),1,-1); 
    deltaMotor=motDeltaFromYaw(yaw,headingTarget,yawSpeedTarget);   
    //outSpeed[0]=manualSpeed[0];
    if((autoSpeed[1]+abs(deltaMotor))>256) autoSpeed[1]=256-abs(deltaMotor); 
    if((autoSpeed[1]-abs(deltaMotor))<-256) autoSpeed[1]=-256+abs(deltaMotor);
    outSpeed[0]=autoSpeed[1]+deltaMotor;
    outSpeed[1]=autoSpeed[1]-deltaMotor;
    if(motorEnable){
      portMotor->setSpeed(constrain(abs(outSpeed[0]),0,255));
      starboardMotor->setSpeed(constrain(abs(outSpeed[1]),0,255));  
      if(outSpeed[1]>0) starboardMotor->run(FORWARD);
      if(outSpeed[1]==0) starboardMotor->run(RELEASE);
      if(outSpeed[1]<0) starboardMotor->run(BACKWARD);
      if(outSpeed[0]>0) portMotor->run(FORWARD);
      if(outSpeed[0]==0) portMotor->run(RELEASE);
      if(outSpeed[0]<0) portMotor->run(BACKWARD);
    } 
    if(rosOn){
      /* 
      char hello[66];//=tempString;
      String tempString=String(joyStickYAnalog); 
      //tempString.toCharArray(hello,66);    
      
      //str_msg.data = hello;
      //chatter.publish( &str_msg );
      //char base_link[] = "/camera_frame";
      // char odom[] = "/odom";      
       //tf.odom->base_link;
      
       t.header.frame_id = odom;
       t.child_frame_id = base_link;
       t.transform.translation.x = 0;     //connect motion;
       t.transform.translation.y = 0;     //connect y motion; 
       t.transform.translation.x = od.pose.pose.position.x;    
       t.transform.translation.y = od.pose.pose.position.y;      
       od.pose.pose.orientation = tf::createQuaternionFromYaw(-yaw*M_PI/180);
       t.transform.rotation = od.pose.pose.orientation;
       t.header.stamp = nh.now();
       broadcaster.sendTransform(t);
            
       od.header.stamp=nh.now();
       od.header.frame_id="/odom";
       od.child_frame_id="/camera_frame";       
       od.pose.pose.position.z=0;       
       //currentRosUploadTime=millis();
       //headingDelta=-(headingCurrent-lastHeading)*(M_PI/180)/(currentRosUploadTime-oldRosUploadTime)*1000; 
       //oldRosUploadTime=currentRosUploadTime;       
       
       od.twist.twist.angular.z=yawDelta*M_PI/180*loopFreq;  //rad/sec
       //Odometry.publish(&od);        
       
       //headingDelta_msg.data=yaw;//headingCurrent;
       headingDelta_pub.publish(&headingDelta_msg);
       */
       
      
      nh.spinOnce();
      
    }
    if(!rosOn){ 
    
    // Serial.println(millis());
    //if(!rosOn) Serial.print(" , ");
    //if(!rosOn) Serial.print("port encoder count = ");
    //if(!rosOn) Serial.print(encoderCount[0]);
    //if(!rosOn) Serial.print(" , star encoder=");
    //if(!rosOn) Serial.print(encoderCount[1]);
    //if(!rosOn) Serial.print(" , star speed=");
    //if(!rosOn) Serial.print(motorSpeed[1],6);
    //*/
    //if(!rosOn) Serial.print("Mag( x,y,z), gz = ");
    //if(!rosOn) Serial.print(mx-magXOffset);
    //if(!rosOn) Serial.print(" , ");
    //if(!rosOn) Serial.println(my-magYOffset);
    //if(!rosOn) Serial.print("button x is ");
    //if(!rosOn) Serial.println(buttonXOn);
    //if(!rosOn) Serial.print(" , ");
    //if(!rosOn) Serial.println(gz);
    //if(!rosOn) Serial.print(" , ")
     Serial.print("yaw,yaw delta=");
     Serial.print(yaw);
     Serial.print(" , ");
     Serial.println(yaw-headingTarget);    
    //if(!rosOn) Serial.print("magYaw,");
    //if(!rosOn) Serial.println(magYaw);
    //if(!rosOn) Serial.print(" , ")
    
     Serial.print("motor delta =");
     Serial.println(deltaMotor);
    //if(!rosOn) Serial.print(" , ");
    //if(!rosOn) Serial.print(manualSpeed[1]);
    //if(!rosOn) Serial.print(" , ");
     Serial.print("Joystic X,Y = ");
     Serial.print(joyStickXAnalog);
     Serial.print(" , ");
     Serial.println(joyStickYAnalog);
     
     Serial.print("Measured speed=");    
     Serial.print(motorSpeed[0]);
     Serial.print(" , ");
     Serial.println(motorSpeed[1]);
     
     Serial.print("Position x,y = ");    
     Serial.print(position[0]);
     Serial.print(" , ");
     Serial.println(position[1]);
     
     Serial.print("outSpeed=");   
     Serial.print(outSpeed[0]);
     Serial.print(" , ");
     Serial.println(outSpeed[1]);
     
    
    }
  } 
}
