#include <TimerOne.h>

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

float counterA=0;
float counterB=0;

void docountA()  // counts from the speed sensor
{
  counterA++;// increase +1 the counter value
  if (counterA == 22){
    analogWrite(enA, 0);
    delay(1000);
  }
} 

void docountB()
{
  counterB++;
  if (counterB == 22){
    analogWrite(enB, 0);
    delay(1000);
  }
}
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer

  Serial.println("----------------------------------");
  Serial.print("MotorA Count : ");
  Serial.print(counterA);
  Serial.print(" Motor Speed: "); 
  float rotationA = (counterA / 20);  // divide by number of holes in Disc
  Serial.print(rotationA);  
  Serial.println(" Rotation per seconds"); 

  Serial.print("MotorB Count : ");
  Serial.print(counterB);
  Serial.print(" Motor Speed: "); 
  float rotationB = (counterB / 20);  // divide by number of holes in Disc
  Serial.print(rotationB);  
  Serial.println(" Rotation per seconds"); 

  
  counterA=0;  //  reset counter to zero
  counterB=0;
  
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

void setup()
{

  Serial.begin(9600);
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ir_a, INPUT);
  pinMode(ir_b, INPUT);

  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(0, docountA, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(1, docountB, RISING);
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void forward(int delay_sec){
  //turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 85);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 80);
  delay(delay_sec);
}

void backward(int delay_sec){
  //turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 200);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 200);
  delay(delay_sec);
}

void turn_left(int delay_sec){
  //turn off motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 200);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 200);
  delay(delay_sec);
}

void turn_right(int delay_sec){
  //turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 200);
  // turn off motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 200);
  delay(delay_sec);
}

void stopped(int delay_sec){
  //turn off motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  // turn off motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(delay_sec);
}

void loop()
{
  forward(5000);
  //stopped(1000);
  //backward(3000);
  //stopped(1000);
  //turn_left(3000);
  //stopped(1000);
  //turn_right(3000);
  //stopped(5000);
}

