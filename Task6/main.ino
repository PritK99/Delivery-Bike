/*
 * Team Id: 3656
 * Author List: Kavan Gandhi, Mihir Rathod, Prit Kanadiya, Raghav Agarwal
 * Filename: main.ino
 * Theme: Delivery Bike
 * Functions: nidec_motor_init(), nidec_motor_control(), nidec_motor_brake(), timer1_init(), void servo_init(),     servo_move(), rencoderL(), Tuning(), dc_motor_init(), dc_motor_forward(), dc_motor_backward(), printValues().
 * Global Variables: x, t2_w1, t2_w2, theta, prev_theta, prev_alpha, pwm, w2, w1, z, target_vel, loop_count, lt, prev_w2, random_angle, count, alpha, brake, go, cw, encodPinAL, encodPinBL, MAX_RPS, MAX_TORQUE, enA, in1, in2, encoderPosAL, prev_encoderPosAL, K[4], pos, Servo1, nextpos, servoPin, prev_proportional, proportional, derivative, integral, turn, lsa[5], kp, ki, kd
 */
#include "Wire.h"
#include <MPU6050_light.h>
#include <math.h>
#include <Servo.h>

MPU6050 mpu(Wire);

float x = 0.0; // x is angle of the bot in degrees
float t2_w1 = 0.0;
float t2_w2 = 0.0;
float prev_theta = 0.0;
float prev_alpha = 0.0;
int pwm = 0;
float w2 = 0;
float w1 = 0;
float z = 0; // z is used to record the equilbrium point of bike dynamically during first 2 seconds.
float target_vel = 0.0;
int loop_count = 0; // loop_count keeps track of number of times loop() has been called and is used to give an external delay in the form of lt loops
int lt = -1;        // lt controls the number of loops the code has to wait before executing loop() function again. when lt = -1, it indicates no external delay has been provided between calling loop() function.
float prev_w2 = 0;
volatile long int random_angle = 0;
float alpha = 0;

#define brake 8      // brake=0, go=1
#define cw 4         // cw=1, ccw=0
#define rpm 9        //
#define encodPinAL 2 // encoder A pin (INT4)
#define encodPinBL 3 // encoder B pin (INT5)
#define MAX_RPS 314
#define MAX_TORQUE 255

#define enA 7
#define in1 22
#define in2 23

volatile int encoderPosAL = 0;
volatile int prev_encoderPosAL = 0; // left count

float k[4] = {-11.582760, -1.179519, -0.017239, -0.020962}; // negative sign in first two values mean the reaction wheel will move opposite to the direction of fall

float prev_proportional = 0, proportional = 0, derivative = 0, integral = 0, turn = 0;
int lsa[5] = {0};
float kp = 0, ki = 0, kd = 0;

// Create a servo object
Servo Servo1;
int pos = 0;
int nextpos = 0;
#define servoPin 6 // Declare the Servo pin

/////////////NIDEC Motor//////////////
/*
 * Function Name: nidec_motor_init
 * Input: None
 * Output: None
 * Logic: Intializes the nidec motor
 * Example Call: nidec_motor_init()
 */
void nidec_motor_init()
{
    pinMode(brake, OUTPUT);
    pinMode(cw, OUTPUT);
    pinMode(rpm, OUTPUT);

    digitalWrite(brake, HIGH); // go=1
    digitalWrite(cw, LOW);     // direction ccw
    analogWrite(rpm, 255);
}

/*
 * Function Name: nidec_motor_control
 * Input: pwm
 * Output: None
 * Logic: Moves the nidec motor as per the pwm given to it
 * Example Call: nidec_motor_control(100)
 */
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

/*
 * Function Name: nidec_motor_brake
 * Input: None
 * Output: None
 * Logic: Stops the nidec motor
 * Example Call: nidec_motor_brake()
 */
void nidec_motor_brake()
{
    digitalWrite(brake, LOW); // go=1
    analogWrite(rpm, 255);
}
//////////////////////////////////////

/////////////Timer1 ISR for IMU///////
/*
 * Function Name: timer1_init
 * Input: None
 * Output: None
 * Logic: Intializes the timer 1
 * Example Call: timer1_init()
 */
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

///////////Servo Motor//////////////
/*
 * Function Name: servo_init
 * Input: None
 * Output: None
 * Logic: Intializes the servo motor
 * Example Call: servo_init()
 */
void servo_init()
{
    Servo1.attach(servoPin);
    pos = 125;
    Servo1.write(pos); //
}

/*
 * Function Name: servo_move
 * Input: nextpos
 * Output: None
 * Logic: Moves the servo to the position required
 * Example Call: servo_move(90)
 */
void servo_move(int nextpos)
{
    //  nextpos=nextpos+125;
    if (pos < nextpos)
    {
        for (int i = pos; i <= nextpos; i += 1)
        {
            Servo1.write(i);
            delay(10);
        }
    }
    else if (pos > nextpos)
    {
        for (int i = pos; i >= nextpos; i -= 1)
        {
            Servo1.write(i);
            delay(10);
        }
    }
    else
    {
        pos = nextpos;
    }
    pos = nextpos;
}
////////////////////////////////////

/////////ENCODER////////
/*
 * Function Name: rencoderL
 * Input: None
 * Output: None
 * Logic: Increments and decrements value of encoderPosAL as per the motion of reaction wheel
 * Example Call: rencoderL()
 */
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
////////////////////////////////////

/*
 * Function Name: setup
 * Input: None
 * Output: None
 * Logic: Performs intial setup by intializing all the peripheral hardware such as DC motor, Servo motor, Nidec motor, encoders etc.
 * Example Call: setup()
 */
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
    dc_motor_init();
    Serial.println("DC motor begin done!\n");
    nidec_motor_init();
    // Serial.println("NIDEC initialized\n");
    timer1_init();
    servo_init();
    Serial.println("Servo initialized\n");
    pinMode(encodPinAL, INPUT);
    pinMode(encodPinBL, INPUT);
    digitalWrite(encodPinAL, HIGH);                               // turn on pullup resistor
    digitalWrite(encodPinBL, HIGH);                               // turn on pullup resistor
    attachInterrupt(digitalPinToInterrupt(2), rencoderL, RISING); // 18 as int
}

/*
 * Function Name: Tuning
 * Input: None
 * Output: None
 * Logic: To dynamically tune the values of K matrix during run-time of bot
 * Example Call: Tuning()
 */
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
            k[0] = val * 0.1;

        printValues();
        Serial.println("1");
        break;
    case 2:
        if (cmd == '+')
            k[1] += val;
        if (cmd == '-')
            k[1] -= val;
        if (cmd == '=')
            k[1] = val * 0.1;
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

/*
 * Function Name: dc_motor_init
 * Input: None
 * Output: Initializes the Rear DC Motor
 * Logic: Initialize motors
 * Example Call: dc_motor_init()
 */
void dc_motor_init()
{
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // Turn off motors - Initial state
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

/*
 * Function Name: dc_motor_forward
 * Input: enablePWM
 * Output: Runs the motor in forward direction
 * Logic: Runs the motor in forward direction as per the PWM given
 * Example Call: dc_motor_forward(90)
 */
void dc_motor_forward(int enablePWM)
{
    // Serial.println(run_rear_motor);
    analogWrite(enA, enablePWM);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
}

/*
 * Function Name: dc_motor_backward
 * Input: enablePWM
 * Output: Runs the motor in backward direction
 * Logic: Runs the motor in backward direction as per the PWM given
 * Example Call: dc_motor_backward(90)
 */
void dc_motor_backward(int enablePWM)
{
    analogWrite(enA, enablePWM);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

/*
 * Function Name: printValues()
 * Input: None
 * Output: Prints the values of K matrix
 * Logic: For debugging purpose and manipulting K values according to the bike
 * Example Call: printValues()
 */
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

/*
 * Function Name: printLSA()
 * Input: None
 * Output: Prints the values of Line Sensor Array
 * Logic: For debugging purpose
 * Example Call: printLSA()
 */
void printLSA()
{
    Serial.print(lsa[0]);
    Serial.print(lsa[1]);
    Serial.print(lsa[2]);
    Serial.print(lsa[3]);
    Serial.print(lsa[3]);
}

/*
 * Function Name: loop()
 * Input: None
 * Output: None
 * Logic: main driver function
 * Example Call: None
 */
void loop()
{
    if (millis() < 2000)
    {
        z = (x * M_PI / 180) + 0.05;
        Serial.print("Setpoint initiated : ");
        Serial.println(z);
    }
    else if (loop_count++ > lt)
    {
        if (millis() > 18000)
        {
            Serial.println("DC Motor on");
            dc_motor_backward(90);
        }

        Tuning(); // start tuning of bot
        float y_setpoint[4] = {z, 0, 0, 0};

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
            Serial.println("Error in calculating dt_1");
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
            Serial.println("Error in calculating dt_2");
        }

        encoderPosAL = 0;

        float y[4] = {theta, w1, alpha, w2};

        if (theta < 0.5 and theta > -0.5)
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
            nidec_motor_control(pwm * 1.05);
        }
        else
        {
            nidec_motor_brake();
        }

        prev_theta = theta;

        lsa[0] = (analogRead(A0));
        lsa[1] = (analogRead(A1));
        lsa[2] = (analogRead(A2));
        lsa[3] = (analogRead(A3));
        lsa[4] = (analogRead(A4));

        pos = (-5 * lsa[0] + -3 * lsa[1] + 1 * lsa[2] + 3 * lsa[3] + 5 * lsa[4]) / (lsa[0] + lsa[1] + lsa[2] + lsa[3] + lsa[4]);

        prev_proportional = proportional;
        proportional = pos - 1;
        derivative = proportional - prev_proportional;
        integral += proportional;

        turn = proportional * kp + integral * ki + derivative * kd;

        Serial.print("Theta : ");
        Serial.print(theta);
        Serial.print(" EPAL : ");
        Serial.print(encoderPosAL);
        Serial.print(" PWM : ");
        Serial.print(pwm);
        Serial.print(" W2 : ");
        Serial.print(w2);
        Serial.print(" Alpha : ");
        Serial.print(alpha);
        Serial.print(" pos: ");
        Serial.println(pos);
        Serial.print(" nextpos: ");
        Serial.println(nextpos);

        loop_count = 0;
    }
    random_angle = encoderPosAL;
}