
/*
  This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
  It won't work with v1.x motor shields! Only for the v2's with built in PWM
  control

  For use with the Adafruit Motor Shield v2
  ---->	http://www.adafruit.com/products/1438
*/



#include <Adafruit_MotorShield.h>
#include <MadgwickAHRS.h>
#include <Arduino_LSM9DS1.h>
#include <string>
#include <iostream>
//void encoderSetup();

unsigned long microsPerReading, microsPrevious;
Madgwick filter;
bool motorEnable=false;
int staticSpeed=150;
int manualSpeed[2] = {0,0};
int autoSpeed[2]={0,0};
int deltaMotor=0;


float yaw;
float initialYaw;
float mx,my,mz; //units = Deg/second
float gx,gy,gz; //degrees/sec
float magYaw;
float magXOffset=85.91;
float magYOffset=50.02;
float gXOffset=.59;
float gYOffset=1.80;
float gZOffset=.69;
float motorSpeed[2];
int portSpeed=0;
volatile int encoderCount[2] = {0, 0}; //port=0 starboard=1
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
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.setTimeout(1000);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  encoderSetup();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
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
  filter.begin(100);  //10hz
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 100;
  microsPrevious = micros();
  pinMode(LED_BUILTIN,OUTPUT);
  bool ledState=true;
  
  for (int i = 0; i < 600; i++) {
    delay(10);
    digitalWrite(LED_BUILTIN,ledState);
    ledState=!ledState;    
    yaw=updateYaw();
    if(yaw>-1) initialYaw=yaw;
  }

}
//==========================================================================================
void calibrate(){
  float calX[4]={0,0,0,0}; // 0=360deg, 1=90deg, 2=180deg, 3=270deg
  float calY[4]={0,0,0,0};
  float internalYaw;
  float yawPoints[4]={0,0,0,0};
  float calXRange[2]={9999,-9999};// 0=min, 1=max  
  float calYRange[2]={9999,-9999};
  unsigned int startCountMillis=millis();
  //float mx,my,mz;
  float cumMx, cumMy;
  float cumGX,cumGY,cumGZ;
  int   measurements;
  digitalWrite(LED_BUILTIN,true);
  Serial.println("Calibration in progress, do not move bot until instructed");
  measurements=0;
  cumMx=0;
  cumMy=0;
  cumGX=0;
  cumGY=0;
  cumGZ=0;
  while(millis()<startCountMillis+2000){}
  while(millis()<startCountMillis+4000){ //measure 0 heading
   yaw=updateYaw(); 
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
  Serial.println("Please turn bot right to 90 deg and wait");
  while(millis()<startCountMillis+6000){
   yaw=updateYaw(); 
   if (yaw>-1.0) {    
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
   }  
  }
  digitalWrite(LED_BUILTIN,true);
  Serial.println("90 degree Measurement in progress do not move bot until instructed");
  while(millis()<startCountMillis+8000){ //measure 90 heading
  yaw=updateYaw(); 
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
  Serial.println("Please turn bot right to 180 deg and wait");
  while(millis()<startCountMillis+10000){
  yaw=updateYaw(); 
   if (yaw>-1.0) {     
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
   }  
  } 
  Serial.println("180 degree Measurement in progress do not move bot until instructed");
  digitalWrite(LED_BUILTIN,true);
  while(millis()<startCountMillis+12000){ //measure 180 heading
   yaw=updateYaw(); 
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
  Serial.println("Please turn bot right to 270 deg and wait");
  while(millis()<startCountMillis+14000){
  yaw=updateYaw(); 
   if (yaw>-1.0) {     
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
   }  
  }
  Serial.println("270 degree Measurement in progress do not move bot until instructed");
  digitalWrite(LED_BUILTIN,true);
  while(millis()<startCountMillis+16000){  //measure 270 heading
  yaw=updateYaw(); 
   if (yaw>-1.0) {
     cumMx+=mx;
     cumMy+=my;
     measurements++;
     if(mx>calXRange[1]) calXRange[1]=mx;
     if(my>calYRange[1]) calYRange[1]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
     if(mx<calXRange[0]) calXRange[0]=mx;
     if(my<calYRange[0]) calYRange[0]=my; 
   }  
  }
  digitalWrite(LED_BUILTIN,false);
  Serial.println("Please turn bot right to 360 deg and wait");
  while(millis()<startCountMillis+18000){
  yaw=updateYaw(); 
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
  Serial.println("Final Measurement in progress do not move bot until instructed");
  digitalWrite(LED_BUILTIN,true);
  while(millis()<startCountMillis+24000){  //measure initial yaw 
    yaw=updateYaw();
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
  Serial.println("Calibration complete Thank You!");
  Serial.print("recalibrated Mag offset (x,Y) Gz =");
  Serial.print(magXOffset);
  Serial.print(", ");
  Serial.println(magYOffset);
  Serial.print("recalibrated G offset (x,y,z)");
  Serial.print(gXOffset);
  Serial.print(", ");
  Serial.print(gYOffset); 
  Serial.print(", ");  
  Serial.println(gZOffset);  
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
  Serial.println("do you want to save these values? Y or N");
  String newString=Serial.readString();
  if (newString="y") {
    Serial.print("OK, save calibration_data,");
    Serial.print(magXOffset);
    Serial.print(",");
    Serial.print(magYOffset);
    Serial.print(",");
    Serial.print(gXOffset);
    Serial.print(",");
    Serial.print(gYOffset);
    Serial.print(",");
    Serial.println(gZOffset);
    
  }
}
//===========================================================================================
int motDeltaFromYaw(float yaw, float HeadingTarget) {
  float yawSpeedTargetMagnitude=10; 
  float yawSpeedTarget=0;
  float yawDeadBand=3;
  float yawSPGain=5; 
  float yawErr=HeadingTarget-yaw;
  static float yawOld=0;
  float yawDelta=yaw-yawOld;
  if (yawErr>180) yawErr=yawErr-360;
  if (yawErr<-180) yawErr=yawErr+360;
  if (abs(yawErr)<yawDeadBand) return 0;
  if (yawErr<yawDeadBand) yawSpeedTarget=-yawSpeedTargetMagnitude;
  if (yawErr>yawDeadBand) yawSpeedTarget=yawSpeedTargetMagnitude;
  float yawDeltaErr=yawSpeedTarget-yawDelta;
  if (yawDeltaErr>180) yawDeltaErr=yawDeltaErr-360;
  if (yawDeltaErr<-180) yawDeltaErr=yawDeltaErr+360;
  int motorDeltaOut=round(yawDeltaErr*yawSPGain);  
  return motorDeltaOut; 
 
}
//==========================================================================================
float updateYaw() {
  static float ax, ay, az;    //in Gs 
  //static float gx, gy, gz;    //in Deg/second
  //static float mx, my, mz;
  static unsigned long microsPrevious=micros();
  unsigned long microsNow=micros();
  unsigned long microsPerReading=10000;  // 100hz
  
  float internalYaw;
  // check if it's time to read data and update the filter
  if (microsNow >= microsPrevious+microsPerReading)
  {
    microsPrevious = microsNow;

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

    filter.update(gx-gXOffset, -gy+gYOffset, gz-gZOffset, ax, -ay, az, -(my - magYOffset), (mx - magXOffset), mz); //manualy measured offsets for x and y magnatome
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
  float speedPGain=.02;
  float speedIGain=.12;
  
  int integratorDepth=15;
  float integratorArray[integratorDepth]={ };
  float integratorSum=0;  
  int lowSpeedOffset=10; //min motor dac to get reliable movment
  float calculatedSpeed=(tach0Input*motorSpeed[0]+tach1Input*motorSpeed[1])/(abs(tach0Input)+abs(tach1Input));  
  float err=targetSpeed*13.0f-calculatedSpeed; //motorSpeed[motNo];  
  static int i=0;
  integratorArray[i]=err;
  if (i<integratorDepth) i++;
  if (i==integratorDepth) i=0;
  integratorSum=0;
  for (int j=0;j<integratorDepth;j++) integratorSum+=integratorArray[j];  
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
  unsigned long loopRateMicros = 100;  //micros
  float speedOut = motorSpeed[motNo];
  unsigned long currentMicros = millis();
  static unsigned long lastCount = currentMicros;
  static int lastEncoderCount[2] = {encoderCount[0], encoderCount[1]};
  static unsigned long whenLastEncoderChange[2] = {millis(), millis()};
  //if (lastCount<currentMicros) lastCount=currentMicros+loopRateMicros;
  //if (currentMicros  > loopRateMicros+lastCount)
  {
    //Serial.println (micros());
    lastCount = currentMicros + loopRateMicros;
    if (encoderCount[motNo] != lastEncoderCount[motNo])
    {
      speedOut = 10e3f * float(encoderCount[motNo] - lastEncoderCount[motNo]) / float(currentMicros - whenLastEncoderChange[motNo]);
      whenLastEncoderChange[motNo] = currentMicros;
      lastEncoderCount[motNo] = encoderCount[motNo];
    }
    else if (currentMicros  >= 300+whenLastEncoderChange[motNo]) {
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
  unsigned long loopRateMicros = 100000;
  int outSpeed[2]={0,0};
  static char serialBuffer[100];
  static int bufferPos=0;
  String inString="";
  char serialInByte;
  bool keyboardInputReady=false;
  bool singleKeyCommandSkip=false;
  yawTemp = updateYaw();
    if (yawTemp > -1.0) {    //???????
      yaw = yawTemp - initialYaw;
      if (yaw < 0.0) yaw = yaw + 360.0;
    }
  if (micros() > loopRateMicros+lastClock)
  {
    lastClock = lastClock + loopRateMicros;
    //Serial.print("main loop micros=");
    //Serial.println(lastClock);
    
    //byte inString[116];
    String inString;
    
    while (Serial.available() > 0) 
    {      
      serialInByte=char(Serial.read());
      serialBuffer[bufferPos]=serialInByte;
      bufferPos++;
      if (bufferPos>63) {
        bufferPos=0;
        inString="";
      }
      //Serial.print("bufferPos =");
      //Serial.println(bufferPos);
      if (serialInByte=='$') { 
        
        inString="";       
        for(int i=0;i<bufferPos-1;i++){
          inString+=serialBuffer[i];
        } 
        inString+='\0';
        keyboardInputReady=true;         
        break;
      }      
    }
      //if (inByte
      //byte inString[116];
      //String inString = String(200);
      //inString=Serial.readStringUntil('\n');  
      //inString=inString+'\n';    
      //if (inString.length()>0)
    if (keyboardInputReady)  {        
       //Serial.print("Read Key = ");
       //Serial.println(inString);
       //Serial.println(bufferPos);
       //Serial.println(serialInByte);
       //Serial.print(serialBuffer[0]);
       //Serial.print(serialBuffer[1]);
       //Serial.println(serialBuffer[2]);
       Serial.println(inString.substring(0,5));
       //inString="";
      if (inString.substring(0,5)=="start"){
         Serial.println("Read Key = start");
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
          break;
        case 'a': //left
          manualSpeed[0]=-staticSpeed;
          manualSpeed[1]=staticSpeed;          
          break;
        case 'd': //right
          manualSpeed[0]=staticSpeed;
          manualSpeed[1]=-staticSpeed;
           break;
        case 'x': //backup
          manualSpeed[0]=-staticSpeed;
          manualSpeed[1]=-staticSpeed;          
          break;
        case '+': //increase speed;
          Serial.println("current speed = ");
          Serial.println(staticSpeed);          
          if (staticSpeed < 250) staticSpeed += 5;          
          Serial.println("new speed = ");
          Serial.println(staticSpeed);
          manualSpeed[0]=manualSpeed[0]/abs(manualSpeed[0])*staticSpeed;
          manualSpeed[1]=manualSpeed[1]/abs(manualSpeed[1])*staticSpeed;
          break;
        case '-': //decrease speed;
          Serial.println("current speed = ");
          Serial.println(staticSpeed);          
          if (staticSpeed > 5) staticSpeed -= 5;          
          Serial.println("new speed = ");
          Serial.println(staticSpeed);
          manualSpeed[0]=manualSpeed[0]/abs(manualSpeed[0])*staticSpeed;
          manualSpeed[1]=manualSpeed[1]/abs(manualSpeed[1])*staticSpeed;
          break;
       }
      } 
      bufferPos=0; 
      inString="";   
    }
        
    autoSpeed[1]=speedControl(manualSpeed[1],1,1);
    deltaMotor=speedControl(motDeltaFromYaw(yaw,0),1,-1);    
    outSpeed[0]=manualSpeed[0];
    //outSpeed[1]=manualSpeed[1];
    
    //deltaMotor=motDeltaFromYaw(yaw,0);
    outSpeed[0]=autoSpeed[1]+deltaMotor;
    outSpeed[1]=autoSpeed[1]-deltaMotor;
    if(motorEnable){
      portMotor->setSpeed(uint8_t(abs(outSpeed[0])));
      starboardMotor->setSpeed(uint8_t(abs(outSpeed[1])));  
      if(outSpeed[1]>0) starboardMotor->run(FORWARD);
      if(outSpeed[1]==0) starboardMotor->run(RELEASE);
      if(outSpeed[1]<0) starboardMotor->run(BACKWARD);
      if(outSpeed[0]>0) portMotor->run(FORWARD);
      if(outSpeed[0]==0) portMotor->run(RELEASE);
      if(outSpeed[0]<0) portMotor->run(BACKWARD);
    } 

    for (int i = 0; i < 2; i++)
    {
      motorSpeed[i] = measureSpeed(i); 
    } 
    
    Serial.print(millis());
    Serial.print(" , ");
    //Serial.print("port encoder count = ");
    //Serial.print(encoderCount[0]);
    //Serial.print(" , star encoder=");
    //Serial.print(encoderCount[1]);
    //Serial.print(" , star speed=");
    //Serial.print(motorSpeed[1],6);
    //*/
    //Serial.print("Mag( x,y,z), gz = ");
    //Serial.print(mx);
    //Serial.print(" , ");
    //Serial.print(my);
    //Serial.print(" , ");
    //Serial.println(mz);
    //Serial.print(" , ");
    //Serial.println(gz);
    //Serial.print(" , ")
    Serial.print("yaw,");
    Serial.println(yaw);    
    //Serial.print("magYaw,");
    //Serial.println(magYaw);
    //Serial.print(" , ")
    
    //Serial.print("motor delta,");
    //Serial.println(deltaMotor);
    //Serial.print(" , ");
    //Serial.print(manualSpeed[1]);
    //Serial.print(" , ");
    Serial.print("outSpeed=");
    //Serial.print(" , ");
    Serial.print(outSpeed[0]);
    Serial.print(" , ");
    Serial.println(outSpeed[1]);
    //
    
  } 
}