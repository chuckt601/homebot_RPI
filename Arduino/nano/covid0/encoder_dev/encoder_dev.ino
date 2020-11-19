
#define __nRF528x__
#define Pin_starboardP 2
#define Pin_starboardQ 3

volatile int encoderCount=0;
volatile int state=0;
volatile bool encoderErr=false;

void Encodersetup(int Pin_P, int Pin_Q,) {
  // put your setup code here, to run once
  
  pinMode(Pin_starboardP,INPUT);
  pinMode(Pin_starboardQ,INPUT);
  attachInterrupt(digitalPinToInterrupt(Pin_starboardP), ISRSPR, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin_starboardP), ISRSPF, FALLING);
  attachInterrupt(digitalPinToInterrupt(Pin_starboardQ), ISRSQR, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin_starboardQ), ISRSQF, FALLING);
  attachInterrupt(digitalPinToInterrupt(Pin_portP), ISRPPR, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin_portP), ISRPPF, FALLING);
  attachInterrupt(digitalPinToInterrupt(Pin_portQ), ISRPQR, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin_portQ), ISRPQF, FALLING);
}

void ISRSPR(){
  if (digitalRead(Pin_starboardQ)==LOW)encoderCount++;
  else encoderCount--;
  return;   
}
void ISRSPF(){
  if (digitalRead(Pin_starboardQ)==LOW)encoderCount--;
  else encoderCount++;
  return;  
}
void ISRSQR(){
  if (digitalRead(Pin_starboardP)==LOW)encoderCount--;
  else encoderCount++;
  return;   
}
void ISRSQF(){
  if (digitalRead(Pin_starboardP)==LOW) encoderCount++;
  else encoderCount--;
  return;  
}
void ISRPPR(){
  if (digitalRead(Pin_portQ)==LOW)encoderCount++;
  else encoderCount--;
  return;   
}
void ISRPPF(){
  if (digitalRead(Pin_portQ)==LOW)encoderCount--;
  else encoderCount++;
  return;  
}
void ISRPQR(){
  if (digitalRead(Pin_portP)==LOW)encoderCount--;
  else encoderCount++;
  return;   
}
void ISRPQF(){
  if (digitalRead(Pin_portP)==LOW) encoderCount++;
  else encoderCount--;
  return;  
}

 
}
void loop() {
  // put your main code here, to run repeatedly:
// int state=0;
 Serial.print("encoder count = ");
 Serial.println(encoderCount);
 Serial.print("encoder state = ");
 Serial.println(state);
 if (encoderErr){
  encoderErr=false;
  Serial.println("error");
 }
 //ISR2();
 delay(10);
//      break;
}
