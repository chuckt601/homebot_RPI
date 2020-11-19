
#define __nRF528x__
#define Pin_starboardP 2
#define Pin_starboardQ 3
#define Pin_portP 4
#define Pin_portQ 5

//volatile int state=0;
//volatile bool encoderErr=false;

void encoderSetup() {
  // put your setup code here, to run once
  
  pinMode(Pin_starboardP,INPUT);
  pinMode(Pin_starboardQ,INPUT);
  pinMode(Pin_portP,INPUT);
  pinMode(Pin_portQ,INPUT);
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
  if (digitalRead(Pin_starboardQ)==LOW)encoderCountStarboard++;
  else encoderCountStarboard--;
  return;   
}
void ISRSPF(){
  if (digitalRead(Pin_starboardQ)==LOW)encoderCountStarboard--;
  else encoderCountStarboard++;
  return;  
}
void ISRSQR(){
  if (digitalRead(Pin_starboardP)==LOW)encoderCountStarboard--;
  else encoderCountStarboard++;
  return;   
}
void ISRSQF(){
  if (digitalRead(Pin_starboardP)==LOW) encoderCountStarboard++;
  else encoderCountStarboard--;
  return;  
}
void ISRPPR(){
  if (digitalRead(Pin_portQ)==LOW)encoderCountPort++;
  else encoderCountPort--;
  return;   
}
void ISRPPF(){
  if (digitalRead(Pin_portQ)==LOW)encoderCountPort--;
  else encoderCountPort++;
  return;  
}
void ISRPQR(){
  if (digitalRead(Pin_portP)==LOW)encoderCountPort--;
  else encoderCountPort++;
  return;   
}
void ISRPQF(){
  if (digitalRead(Pin_portP)==LOW) encoderCountPort++;
  else encoderCountPort--;
  return;  
}
