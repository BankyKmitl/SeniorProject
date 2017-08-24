#include <TimerOne.h>

//set the desired speed here
unsigned int desired_speed=1; 
unsigned int desired_speed2=1;

//variables for the PID controller
unsigned int measured_speed=0; 
unsigned int measured_speed2=0;
unsigned int total_pulses=0; 
unsigned int total_pulses2=0;

unsigned int set_power=0;
unsigned int set_power2=0;
unsigned int maximum_power=255;
unsigned int maximum_power2=255;
unsigned int minimum_power=20;
unsigned int minimum_power2=20;

//tune these PID coefficients
float Kp=16;
float Ki=0.125;
float Kd=0.125;

float Kp2=16;
float Ki2=0.125;
float Kd2=0.125;

int error=0;
int prev_error=0;
int integral=0;
int diff=0;

int error2=0;
int prev_error2=0;
int integral2=0;
int diff2=0;

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 6;
int in1 = 8;
int in2 = 10;
// motor two
int enB = 5;
int in3 = 7;
int in4 = 9;
// ir sensor
int ir_b = 2;
int ir_a = 3;

void docount()  // counts from the speed sensor
{
  measured_speed++;  // increase +1 the measured_speed value
  total_pulses += measured_speed;
    if (measured_speed == 22){
    analogWrite(enA, 0);
    delay(1000);
  }
} 
void docount2()  // counts from the speed sensor
{
  measured_speed2++;  // increase +1 the measured_speed value
  total_pulses2 += measured_speed2;
      if (measured_speed2 == 22){
    analogWrite(enA, 0);
    delay(1000);
  }
}  
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  
  error = measured_speed - desired_speed;
  integral += (error + prev_error)* Ki;
  diff = (error - prev_error)* Kd;

  if (set_power < maximum_power) {set_power -= ((error * Kp) + integral + diff);
  }
  else {set_power =minimum_power; integral =0;};

  error2 = measured_speed2 - desired_speed2;
  integral2 += (error2 + prev_error2)* Ki;
  diff2 = (error2 - prev_error2)* Kd;

  if (set_power2 < maximum_power2) set_power2 -= ((error2 * Kp) + integral2 + diff);
  else {set_power2 = minimum_power2; integral2 = 0;};

  prev_error = error;
  prev_error2 = error2;

  Serial.print("MotorA speed : ");
  Serial.println(measured_speed);  

  Serial.print("MotorB speed : ");
  Serial.println(measured_speed2);
  
  Serial.println("------------------------------------------"); 

  measured_speed=0;  //  reset measured_speed to zero
  measured_speed2=0;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

void setup() {
  // put your setup code here, to run once:
// set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ir_a, INPUT);
  pinMode(ir_b, INPUT);
  
  Serial.begin(9600);
  enable_interrupt();  
}
void enable_interrupt()
{
  Timer1.initialize(100000); // set timer for 0.1 sec
  attachInterrupt(0, docount, RISING);  // increase measured_speed when speed sensor pin goes High
  attachInterrupt(1, docount2, RISING);  // increase measured_speed when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void disable_interrupt()
{
  detachInterrupt(digitalPinToInterrupt(2));  // increase measured_speed when speed sensor pin goes High
  detachInterrupt(digitalPinToInterrupt(3));  // increase measured_speed when speed sensor pin goes High
  Timer1.detachInterrupt(); // enable the timer
}


void demoStraightPID()
{
// this function will run the motors in a straight line at a fixed speed

//set the desired speed here
  desired_speed=1; 
  desired_speed2=1;

  
  unsigned int lpower = (set_power);
  unsigned int lpower2 = (set_power2);  
  
  analogWrite(enA, lpower);
  analogWrite(enB, lpower2);
  
// turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
// turn on motor B: faulty, cannot reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
}

void demoStop()
{
 // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
}

void loop() {
  // put your main code here, to run repeatedly:
  //disable_interrupt();
  //demoOne();
//  demoTwo();
  demoStraightPID();
 // delay(5000);
}
