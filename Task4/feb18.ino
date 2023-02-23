#include "Wire.h"
#include <MPU6050_light.h>
//#include <Servo.h> 
#include <math.h>

MPU6050 mpu(Wire);
float x=0.0;
float t2_w1 = 0.0;
float t2_w2 = 0.0;
float prev_theta = 0.0;
float prev_alpha = 0.0;
int pwm = 0;
float w2=0;
float w1 =0;
float target_vel = 0.0;
int loop_count=0;
int lt=10;
float prev_w2=0;
volatile long int random_angle=0;

//Declare NIDEC Motor pins
#define brake         8  //brake=0, go=1
#define cw            4  //cw=1, ccw=0
#define rpm           9  //PWM=255=stop, PWM=0=max_speed  
#define encodPinAL      2                      // encoder A pin (INT4) 
#define encodPinBL      3                      // encoder B pin (INT5)
#define MAX_RPS 300
#define MAX_TORQUE 255
volatile int encoderPosAL = 0; 
volatile int prev_encoderPosAL = 0;  // left count
float y_setpoint[4] = {-0.0,0,0,0};
//float k[4] =  {-72.909996   ,-7.821821   ,-0.100000  , -0.121669};

//float k[4] =  {-1.264988369  ,-0.128831410 , -0.000100000 , -0.000249496};
//  float k[4] =  {-7.330547 , -0.756507 , -0.010000,  -0.012274};
//float k[4] = {-2.93432096,  -0.29697906,  -0.00100000,  -0.00343115};
//  float k[4] = {-1.41588518,  -0.14220266,  -0.00014142 , -0.00053840};
//  float k[4] = {-1.49535030 , -0.15029910  ,-0.0  ,-0.00071472};
//float k[4] = {-1.56106289 , -0.15700093 , -0.000 , -0.00081634};
float k[4] = {-21.252810   ,-2.166230 ,  -0.00 ,  -0.040343};

//float k[4] =  {-0.8  ,-0.15 , -0.000100000 , -0.000249496};

// Create a servo object
//Servo Servo1;
int pos=0;
#define servoPin      6   // Declare the Servo pin


// Declare the DC Motor Pins
#define enA           7
#define in1           22
#define in2           23


/////////////NIDEC Motor//////////////
void nidec_motor_init()
{ 
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(rpm, OUTPUT);
  
  digitalWrite(brake, HIGH);  //go=1
  digitalWrite(cw, LOW);      //direction ccw
  analogWrite(rpm, 255);
}

void nidec_motor_control(int pwm) 
{ 
  if (pwm < 0) 
  { digitalWrite(cw, HIGH);
    pwm = -pwm;} 
  else { digitalWrite(cw, LOW); }
  analogWrite(rpm, 255 - pwm);
}

void nidec_motor_brake() 
{ 
  digitalWrite(brake, LOW);  //go=1
  analogWrite(rpm, 255);
}
//////////////////////////////////////



/////////////Timer1 ISR for IMU///////
void timer1_init()
{
    cli(); //Clears the global interrupts
    TIMSK1 = 0x01; //timer5 overflow interrupt enable
    TCCR1B = 0x00; //stop
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    TCCR1A = 0x00;
    TCCR1C = 0x00;
    TCCR1B = 0x02; //start Timer, prescaler 8
    sei();   //Enables the global interrupts
}

ISR (TIMER1_OVF_vect)
{
    sei();  
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    mpu.update();
    cli();
    x=mpu.getAngleX();
    
//    Serial.println(x);
}
//////////////////////////////////////


///////////Servo Motor//////////////
//void servo_init()
//{
//  Servo1.attach(servoPin);
//  pos=90;
//  Servo1.write(pos);
//}
//void servo_move(int nextpos)
//{ 
//  if(pos<nextpos)
//  { for(int i=pos;i<=nextpos;i+=1)
//    { Servo1.write(i);
//      delay(10);}
//  }
//  else if(pos>nextpos)
//  { for(int i=pos;i>=nextpos;i-=1)
//    { Servo1.write(i);
//      delay(10); }
//  }
//  else  { pos=nextpos; }
//  pos=nextpos;
//} 
////////////////////////////////////


/////////////DC Motor//////////////
void dc_motor_init()
{
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
void dc_motor_forward(int enablePWM)
{
  //Serial.println(run_rear_motor);
  analogWrite(enA, enablePWM);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void dc_motor_backward(int enablePWM)
{
  analogWrite(enA, enablePWM);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
//////////////////////////////////////

/////////ENCODER////////
void rencoderL()  
{                                  
    if(digitalRead(encodPinBL)==HIGH)
    {
         encoderPosAL++;
    }
    else
    {
         encoderPosAL--;
    }

    
    //Serial.println(encoderPosAL);
}





void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  //Serial.print(F("MPU6050 status: "));
  //Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  //Serial.println("MPU begin done!\n");

  //Serial.println("Begin Device initialization:\n");
  nidec_motor_init();
  //Serial.println("NIDEC initialized\n");
  timer1_init(); 
  //Serial.println("Timer initialized\n");
  //servo_init();
  //Serial.println("Servo initialized\n");
  dc_motor_init();
  //Serial.println("DC Motor initialized\n");
  pinMode(encodPinAL, INPUT);
  pinMode(encodPinBL, INPUT);
  digitalWrite(encodPinAL, HIGH);               // turn on pullup resistor
  digitalWrite(encodPinBL, HIGH);               // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(2), rencoderL, RISING);        // 18 as int
}


int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  int param = Serial.parseInt();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read(); 
  if (!Serial.available()) return 0;
  int val =Serial.parseInt();  
  // get command byte
  //Serial.flush();
  Serial.println("Tuning");
  Serial.println(param);
  switch (param) {
    case  1:
      if (cmd == '+')    k[0] += val;
      if (cmd == '-')    k[0] -= val;
      if (cmd == '=')    k[0] = val;
      
      printValues();
      Serial.println("1");
      break;
    case 2:
      if (cmd == '+')    k[1] += val;
      if (cmd == '-')    k[1] -= val;
      if (cmd == '=')    k[1] = val;
      printValues();
      break;      
    case 3:
      if (cmd == '+')    k[2] += val;
      if (cmd == '-')    k[2] -= val;
      if (cmd == '=')    k[2] = val*0.001;
      printValues();
      break;  
    case 4:
      if (cmd == '+')    k[3] += val;
      if (cmd == '-')    k[3] -= val;
      if (cmd == '=')    k[3] = val*0.001;
      printValues();
      break; 
    case 5:
      printValues();
      break;
      
  }
}

void printValues() {
  Serial.print("K1: "); Serial.print(k[0]);
  Serial.print(" K2: "); Serial.print(k[1]);
  Serial.print(" K3: "); Serial.print(k[2]);
  Serial.print(" K4: "); Serial.println(k[3]);

}


void loop() 
{
  if (loop_count++>lt){
    Tuning();

  
 
  float torque = 0;

  
  float theta = -(x*M_PI/180)-0.05;
  float t1_w1 = (millis())*0.001;
  float dt_1=t1_w1-t2_w1;
  if (dt_1 != 0)
  {
    w1 = (theta - prev_theta)/dt_1;
  }

  if (w1 < 0.01 && w1 > -0.01 )
  {
    w1 = 0 ;
  }
  t2_w1=t1_w1;
  
  //int rotations = int((encoderPosAL-prev_encoderPosAL)/98); 
  
  
//  prev_encoderPosAL=encoderPosAL;
////  w2=pwm/5;
//  if (prev_w2>0 and prev_alpha>alpha ){
//     w2 = ((alpha-prev_alpha+M_PI) + M_PI*2*rotations)/dt;  //error detected
//  }
//  else if ((prev_w2<0 and alpha>prev_alpha)){
//    w2= ((alpha-prev_alpha-M_PI) + M_PI*2*rotations)/dt;
//  }
//  else{
//  w2 = ((alpha-prev_alpha)+M_PI*2*rotations)/dt;
//  }
//  prev_w2=w2;
  float alpha = encoderPosAL*0.06411413579;
  float t1_w2 = (millis())*0.001;
  float dt_2=t1_w2-t2_w2;
  w2 = alpha/dt_2;
  t2_w2 = t1_w2;
  encoderPosAL = 0;
//  prev_alpha = alpha;
  
  float y[4] = {theta, w1, alpha, w2};
//  Serial.print("Y0: ");
//  Serial.print(y[0]);
//  Serial.print("Theta : ");
//  Serial.print(theta);

  
   //float k[4] =  {-1.2 , -0.26764792,  -0.00031623,  -0.00112835};
  //float k[4] =  {-7.330547 , -0.756507 , -0.010000,  -0.012274};
  for(int i = 0; i<4; i++)
  {
    torque = torque - k[i]*(y[i]-y_setpoint[i]);
  }
  target_vel= int(max(min(w2+torque*dt_1/0.0002125,MAX_RPS),-MAX_RPS));
//  target_vel = (w2+torque*dt/0.000375);
//  target_vel=max(min(target_vel,MAX_RPS),-MAX_RPS);
//  
  if(target_vel>0 and target_vel<=MAX_RPS)
  {
    pwm = 255*target_vel/MAX_RPS;
  }
  else if(target_vel>MAX_RPS)
  {
    pwm = 255;   
  }
  else if(target_vel < 0 and target_vel>=-MAX_RPS)
  {
    pwm = 255*target_vel/MAX_RPS;
  }
  else if(target_vel<-MAX_RPS)
  {
    pwm = -255;
  }
  dc_motor_backward(100);
//  if (torque>0)
//  {
//    nidec_motor_control(pwm);
//    //servo_move(0);
////    dc_motor_forward(pwm);
//  }
//  else if (torque<0)
//  {
//    nidec_motor_control(pwm);
//    //servo_move(180);
////    dc_motor_backward(pwm);
//  }
//  else
//  {
//    pwm=0;
//    target_vel=0;
//    nidec_motor_brake();
//    //servo_move(90);
////    dc_motor_backward(0);
//  }
  if (pwm>0 or pwm<0){
  nidec_motor_control(pwm);
  }
  else{
  nidec_motor_brake();
  }
//  nidec_motor_control(255);
  prev_theta = theta;
  //prev_alpha = alpha;
//  printValues();
 
  
//  Serial.print(" Torque : ");
//  Serial.print(torque);
//  Serial.print(" Velocity : ");
//  Serial.print(target_vel);
  Serial.print("Theta : ");
  Serial.print(theta);
//  Serial.print("Encoder: ");
Serial.print(" EPAL : ");
  Serial.print(encoderPosAL);
  Serial.print(" PWM : ");
  Serial.print(pwm);
//   Serial.print(" Torque : ");
//   Serial.println(torque);
  Serial.print(" W2 : ");
  Serial.println(w2);

//  Serial.print(" Rotations : ");
//  Serial.println(rotations);
//  Serial.print("Time: ");
//  Serial.println(dt);
//    Plotter   //
//  Serial.print(theta);
//  Serial.print(" ");
//  Serial.print(" target_vel:");
//  Serial.println(w2);
  //Serial.print(" ");

  
  
  loop_count=0;
//  rotations=0;
  //encoderPosAL=0;
//  delay(100);
  }
  random_angle = encoderPosAL;
}