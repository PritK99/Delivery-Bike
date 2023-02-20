#include "Wire.h"
#include <MPU6050_light.h>
#include <math.h>

MPU6050 mpu(Wire);

float x = 0.0; // x is angle of the bot in degrees
float t2_w1 = 0.0;
float t2_w2 = 0.0;
float prev_theta = 0.0;
float prev_alpha = 0.0;
int pwm = 0;
float w2 = 0;
float w1 = 0;
float target_vel = 0.0;
int loop_count = 0;
int lt = 2;
float prev_w2 = 0;
volatile long int random_angle = 0;
int count = 0;
float alpha = 0 ;

#define brake 8      // brake=0, go=1
#define cw 4         // cw=1, ccw=0
#define rpm 9        // PWM=255=stop, PWM=0=max_speed
#define encodPinAL 2 // encoder A pin (INT4)
#define encodPinBL 3 // encoder B pin (INT5)
#define MAX_RPS 314
#define MAX_TORQUE 255

volatile int encoderPosAL = 0;
volatile int prev_encoderPosAL = 0; // left count
float y_setpoint[4] = {0.0, 0, 0, 0};

float k[4] = {-9.148998,  -0.931186,  -0.013188,  -0.016083}; // negative sign in first two values mean the reaction wheel will move opposite to the direction of fall

int pos = 0;

/////////////NIDEC Motor//////////////
void nidec_motor_init()
{
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(rpm, OUTPUT);

  digitalWrite(brake, HIGH); // go=1
  digitalWrite(cw, LOW);     // direction ccw
  analogWrite(rpm, 255);
}

void nidec_motor_control(int pwm)
{
  if (pwm < 0)
  {
    digitalWrite(cw, HIGH);
    pwm = -pwm;
  }
  else
  {
    digitalWrite(cw, LOW);
  }
  analogWrite(rpm, 255 - pwm);
}

void nidec_motor_brake()
{
  digitalWrite(brake, LOW); // go=1
  analogWrite(rpm, 255);
}
//////////////////////////////////////

/////////////Timer1 ISR for IMU///////
void timer1_init()
{
  cli();         // Clears the global interrupts
  TIMSK1 = 0x01; // timer5 overflow interrupt enable
  TCCR1B = 0x00; // stop
  TCNT1H = 0xA2; // Counter higher 8 bit value
  TCNT1L = 0x3F; // Counter lower 8 bit value
  TCCR1A = 0x00;
  TCCR1C = 0x00;
  TCCR1B = 0x02; // start Timer, prescaler 8
  sei();         // Enables the global interrupts
}

ISR(TIMER1_OVF_vect)
{
  sei();
  TCNT1H = 0xA2; // Counter higher 8 bit value
  TCNT1L = 0x3F; // Counter lower 8 bit value
  mpu.update();
  cli();
  x = mpu.getAngleX();
}
//////////////////////////////////////

/////////ENCODER////////
void rencoderL()
{
  if (digitalRead(encodPinBL) == HIGH)
  {
    encoderPosAL++;
  }
  else
  {
    encoderPosAL--;
  }
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(status);
  while (status != 0)
  {
  } // stop everything if could not connect to MPU6050
  // Serial.println("MPU begin done!\n");

  // Serial.println("Begin Device initialization:\n");
  nidec_motor_init();
  // Serial.println("NIDEC initialized\n");
  timer1_init();
  pinMode(encodPinAL, INPUT);
  pinMode(encodPinBL, INPUT);
  digitalWrite(encodPinAL, HIGH);                               // turn on pullup resistor
  digitalWrite(encodPinBL, HIGH);                               // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(2), rencoderL, RISING); // 18 as int
}

int Tuning()
{
  if (!Serial.available())
    return 0;
  delay(2);
  int param = Serial.parseInt(); // get parameter byte
  if (!Serial.available())
    return 0;
  char cmd = Serial.read();
  if (!Serial.available())
    return 0;
  int val = Serial.parseInt();
  Serial.println("Tuning");
  Serial.println(param);
  switch (param)
  {
  case 1:
    if (cmd == '+')
      k[0] += val;
    if (cmd == '-')
      k[0] -= val;
    if (cmd == '=')
      k[0] = val*0.1;

    printValues();
    Serial.println("1");
    break;
  case 2:
    if (cmd == '+')
      k[1] += val;
    if (cmd == '-')
      k[1] -= val;
    if (cmd == '=')
      k[1] = val*0.1;
    printValues();
    break;
  case 3:
    if (cmd == '+')
      k[2] += val;
    if (cmd == '-')
      k[2] -= val;
    if (cmd == '=')
      k[2] = val * 0.01;
    printValues();
    break;
  case 4:
    if (cmd == '+')
      k[3] += val;
    if (cmd == '-')
      k[3] -= val;
    if (cmd == '=')
      k[3] = val * 0.01;
    printValues();
    break;
  case 5:
    printValues();
    break;
  }
}

void printValues()
{
  Serial.print("K1: ");
  Serial.print(k[0]);
  Serial.print(" K2: ");
  Serial.print(k[1]);
  Serial.print(" K3: ");
  Serial.print(k[2]);
  Serial.print(" K4: ");
  Serial.println(k[3]);
}

void loop()
{
  if (loop_count++ > lt)
  {
    Tuning(); // start tuning of bot

    float torque = 0;
    float theta = (x * M_PI / 180) + 0.05; // we add 0.05 to equalize angles at both falls i.e. to have same angle in terms of magnitude at extreme ends
    float t1_w1 = (millis()) * 0.001;
    float dt_1 = t1_w1 - t2_w1;
    t2_w1 = t1_w1;

    if (dt_1 != 0)
    {
      w1 = (theta - prev_theta) / dt_1;
    }
    else
    {
      Serial.println("Error in calculating dt_1") ;
    }

    alpha = (int((alpha + encoderPosAL * 0.06411413579) * 100) % 314) / 100.0; 

    float t1_w2 = (millis()) * 0.001;
    float dt_2 = t1_w2 - t2_w2;
    t2_w2 = t1_w2;

     if (dt_2 != 0) 
     {
       w2 = (encoderPosAL * 0.06411413579) / dt_2;
     }
     else
     {
       Serial.println("Error in calculating dt_2") ;
     }

    encoderPosAL = 0;

    float y[4] = {theta, w1, alpha, w2};

    if (theta < 0.1 and theta > -0.1)
    {
      for (int i = 0; i < 4; i++)
      {
        torque = torque - k[i] * (y[i] - y_setpoint[i]);
      }
      target_vel = int(max(min(w2 + torque * dt_1 / 0.0002125, MAX_RPS), -MAX_RPS));
      pwm = 255 * target_vel / MAX_RPS;
    }
    else
    {
      pwm = -theta * 1593.75;
    }

    if (pwm > 0 or pwm < 0)
    {
      nidec_motor_control(pwm);
    }
    else
    {
      nidec_motor_brake();
    }

    prev_theta = theta;

    Serial.print("Theta : ");
    Serial.print(theta);
    Serial.print(" EPAL : ");
    Serial.print(encoderPosAL);
    Serial.print(" PWM : ");
    Serial.print(pwm);
    Serial.print(" W2 : ");
    Serial.print(w2);
    Serial.print(" Alpha : ");
    Serial.println(alpha);

    loop_count = 0;
  }
  random_angle = encoderPosAL;
}