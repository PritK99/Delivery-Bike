
#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h> 


MPU6050 mpu(Wire);
float x=0.0;

//Declare NIDEC Motor pins
#define brake         8  //brake=0, go=1
#define cw            4  //cw=1, ccw=0
#define rpm           9  //PWM=255=stop, PWM=0=max_speed  


// Create a servo object
Servo Servo1;
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
    Serial.println(x);
}
//////////////////////////////////////


/////////////Servo Motor//////////////
void servo_init()
{
  Servo1.attach(servoPin);
  pos=90;
  Servo1.write(pos);
}
void servo_move(int nextpos)
{ 
  if(pos<nextpos)
  { for(int i=pos;i<=nextpos;i+=1)
    { Servo1.write(i);
      delay(10);}
  }
  else if(pos>nextpos)
  { for(int i=pos;i>=nextpos;i-=1)
    { Servo1.write(i);
      delay(10); }
  }
  else  { pos=nextpos; }
  pos=nextpos;
}
//////////////////////////////////////


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


void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println("MPU begin done!\n");

  Serial.println("Begin Device initialization:\n");
  nidec_motor_init();
  Serial.println("NIDEC initialized\n");
  timer1_init(); 
  Serial.println("Timer initialized\n");
  servo_init();
  Serial.println("Servo initialized\n");
  dc_motor_init();
  Serial.println("DC Motor initialized\n");
}

void loop() 
{
  int pwm=100;
  
  if (x>10)
  {
    nidec_motor_control(pwm);
    servo_move(0);
    dc_motor_forward(pwm);
  }
  else if (x<-10)
  {
    nidec_motor_control(-pwm);
    servo_move(180);
    dc_motor_backward(pwm);
  }
  else
  {
    nidec_motor_brake();
    servo_move(90);
    dc_motor_backward(0);
  }

}
