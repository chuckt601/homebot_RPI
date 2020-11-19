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
//void encoderSetup();

unsigned long microsPerReading, microsPrevious;
Madgwick filter;
int speed=150;
float yaw;
float initialYaw;
volatile int encoderCountPort=0;
volatile int encoderCountStarboard=0;
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *portMotor = AFMS.getMotor(4);
Adafruit_DCMotor *starboardMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  encoderSetup();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }  
  AFMS.begin(1000);  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  portMotor->setSpeed(speed);
  starboardMotor->setSpeed(speed);
  //myMotor->run(FORWARD);
  // turn on moter
  portMotor->run(RELEASE);
  starboardMotor->run(RELEASE);
  filter.begin(100);
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 /100;
  microsPrevious = micros();
  for (int i=0;i<300;i++) {
    delay(10);
    initialYaw=updateYaw();
    }
  
}


float updateYaw() {
  static float ax, ay, az;
  static float gx, gy, gz;
  static float mx, my, mz;
  unsigned long microsNow; 
  float internalYaw; 
  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) 
  {
    microsPrevious = microsPrevious + microsPerReading; 
  
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
    } 
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);  
    }
    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(mx, my, mz); 
    }
    //Serial.print(mx);
    //Serial.print(" , ");    
    //Serial.print(my);
    //Serial.print(" , "); 
    //Serial.println(mz);   
   
    filter.update(gx, -gy, gz, ax, -ay, az,-(my-40),(mx-92.5), mz);  //manualy measured offsets for x and y magnatometer
  
    //double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    //double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    internalYaw = 360-filter.getYaw();
    return internalYaw;
  }
  return -1000.0;
}

void speedControl(int motNo, float targetSpeed)
 {
   
 } 

float measureSpeed(int motNo)
  {
    static unsigned long lastCount=micros();
    static int lastEncoderCountStarboard=encoderCountStarboard;
    static int lastEncoderCountPort=encoderCountPort;
    if (micros()-lastCount > 10000)
     {
       lastCount=lastCount+10000;
       if(encoderCountStarboard == lastEncoderCountStarboard)
         {
          tbd
         }
       if else
         {
          tbd  
         }
     }
  }

void loop() {
  //uint8_t i;
  float yawTemp;
  float portMotorSpeed;
  float starboardMotorSpeed;  
  yawTemp=updateYaw();
  if (yawTemp>-1.0) {
    yaw=yawTemp-initialYaw;
    if (yaw<0.0) yaw=yaw+360.0;
    }
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    // do something different depending on the character received.  
    // The switch statement expects single number values for each case;
    // in this exmaple, though, you're using single quotes to tell
    // the controller to get the ASCII value for the character.  For 
    // example 'a' = 97, 'b' = 98, and so forth:
    Serial.println(inByte);
    switch (inByte) {
        
    case 'w': //forward
      portMotor->run(FORWARD);
      starboardMotor->run(FORWARD);
      break;
    case 's': //stop
      portMotor->run(RELEASE);
      starboardMotor->run(RELEASE);
      break; 
    case 'a': //left         
      portMotor->run(BACKWARD);
      starboardMotor->run(FORWARD);
      break;   
    case 'd': //left         
      portMotor->run(FORWARD);
      starboardMotor->run(BACKWARD);
      break; 
    case 'x': //left         
      portMotor->run(BACKWARD);
      starboardMotor->run(BACKWARD);
      break; 
    case '+': //increase speed;
      Serial.println("current speed = ");
      Serial.println(speed);
      speed+=5;
      if (speed>254) speed=254;
      Serial.println("new speed = ");
      Serial.println(speed);
      portMotor->setSpeed(speed);
      starboardMotor->setSpeed(speed);  
      break;
    case '-': //decrease speed;
      Serial.println("current speed = ");
      Serial.println(speed);
      speed-=5;
      if (speed<0) speed=0;
      Serial.println("new speed = ");
      Serial.println(speed); 
      portMotor->setSpeed(speed);
      starboardMotor->setSpeed(speed); 
      break;     
    }
  }   
 Serial.print("port encoder count = ");
 Serial.print(encoderCountPort);
 Serial.print(" , starboard encoder count = "); 
 Serial.print(encoderCountStarboard);
 Serial.print(" , yaw = "); 
 Serial.println(yaw);
 

portMotorSpeed=measureSpeed(0);
starboardMotorSpeed=measureSpeed(1);
}

  
