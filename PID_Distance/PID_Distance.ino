#include <TimerOne.h>
#include <PID_v1.h>
#include <Plotter.h>

unsigned int desired_speed=10; 
unsigned int desired_speed2=1;
unsigned int measured_speed=0; 
unsigned int measured_speed2=0;
unsigned int total_pulses=0; 
unsigned int total_pulses2=0;

//Plotter 

//Define Variables we'll be connecting to
double Setpoint, Input_a, Output_a, Input_b, Output_b;

//Specify the links and initial tuning parameters
PID pid_a(&Input_a, &Output_a, &Setpoint,0.005,2,1, DIRECT);
PID pid_b(&Input_b, &Output_b, &Setpoint,0.5,3,1, DIRECT);

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
  total_pulses++;
} 
void docount2()  // counts from the speed sensor
{
  measured_speed2++;  // increase +1 the measured_speed value
  total_pulses2++;
}

void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer

  Input_a = total_pulses;
  Input_b = total_pulses2;
  
  Setpoint = 2000;
  pid_a.Compute();
  pid_b.Compute();
  
  Serial.print("A : ");
  Serial.println(Output_a); 
  Serial.print("B : ");
  Serial.println(Output_b); 
  
  analogWrite(enA, Output_a);
  analogWrite(enB, Output_b);
  
  measured_speed = 0; 
  measured_speed2 = 0;
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
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
// turn on motor B: faulty, cannot reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
  

  pid_a.SetOutputLimits(20, 200);
  pid_a.SetSampleTime(100);
  pid_a.SetMode(AUTOMATIC);

  pid_b.SetOutputLimits(20, 200);
  pid_b.SetSampleTime(100);
  pid_b.SetMode(AUTOMATIC);
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
  


  //Serial.println(Output);
  //analogWrite(enB, lpower2);
  
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

//  Input = measured_speed*10.0/20.0;
//  Setpoint = 15;
//  myPID.Compute();

//  Serial.println(Input);
//  Serial.println(Setpoint);
//  Serial.println(Output);
//  analogWrite(enA, Output);
 // delay(5000);
}
