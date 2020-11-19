//yaw is working, gyro gains too high. how to adjust?


#include <MadgwickAHRS.h>
#include <Arduino_LSM9DS1.h>

//float invSqrt(float x);
unsigned long microsPerReading, microsPrevious;
Madgwick filter;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

 if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ"); 
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
  
  filter.begin(100);
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 /100;
  microsPrevious = micros(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  unsigned long microsNow;
  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
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
    Serial.print(mx);
    Serial.print(" , ");    
    //Serial.print(my);
    //Serial.print(" , "); 
    //Serial.println(mz);   
   
    filter.update(gx, -gy, gz, ax, -ay, az,-(my-40),(mx-92.5), mz);  //manualy measured offsets for x and y magnatometer
  
    //double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    //double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    float  yaw = 360-filter.getYaw();
    Serial.print("yaw = ");
    Serial.print(yaw);
    Serial.print(" , ");    
    Serial.print(mx-92.5);
    Serial.print(" , "); 
    Serial.println(my-40);    
  } 
}
