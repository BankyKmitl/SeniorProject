#include <TimerOne.h>
#include <PID_v1.h>
#include <Ultrasonic.h>

double speed_a = 0, temp_speed_a = 0;
double speed_b = 0, temp_speed_b = 0; 

int hole_a = 0;
int hole_b = 0;

int pwm_a = 0;
int pwm_b = 0;
int min_pwm = 0, max_pwm = 240;

int direction_flag = 1; // 0 = stop, 1 = forward , 2 = backward, 3 = fwd_left, 4 = fwd_right, 5 = bwd_left, 6 = bwd_right

int distance = 0;
int set_distance = 10000;

int ultra_measured = 0;
int set_ultra = 10; // in centimeters
int temp_ultra = 0;
double delta_speed_ref;

double min_speed = 5, max_speed = 20, speed_ref = 10;
double rps_a, rps_b;

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 5;
int in1 = 7;
int in2 = 9;
// motor two
int enB = 6;
int in3 = 8;
int in4 = 10;
// ir sensor
int ir_a = 2;
int ir_b = 3;
// ultrasonic sensor
Ultrasonic ultrasonic(12, 13);

void docount_a()  // counts from the speed sensor
{
  if (direction_flag == 1 || direction_flag == 3 || direction_flag == 4) {
    speed_a++;
    hole_a++;
  } else if (direction_flag == 2 || direction_flag == 5 || direction_flag == 6) {
    speed_a++;
    hole_a--;
  }
}

void docount_b()  // counts from the speed sensor
{
  if (direction_flag == 1 || direction_flag == 3 || direction_flag == 4) {
    speed_b++;
    hole_b++;
  } else if (direction_flag == 2 || direction_flag == 5 || direction_flag == 6){
    speed_b++;
    hole_b--;
  }
}

void timerIsr()
{
  Timer1.detachInterrupt();  //stopped the timer
  rps_a = speed_a / 20.0;
  rps_b = speed_b / 20.0;
  
  //distance control speed
  distance = (hole_a + hole_b) / 2;
  speed_ref = ((set_distance - distance) / 20 ) / 10 ;     // 20 = hole , 10 = sec

  //ultrasonic control speed
  ultra_measured = ultrasonic.distanceRead();
  delta_speed_ref = (((ultra_measured - set_ultra) / float(set_ultra)) - floor((ultra_measured - set_ultra) / set_ultra));
//  Serial.println(delta_speed_ref);
  
  if (temp_ultra > ultra_measured){
    if (ultra_measured % set_ultra == 0 && ultra_measured != set_ultra)
      speed_ref = speed_ref * (delta_speed_ref+1);
    speed_ref = speed_ref * delta_speed_ref;}
    
  else if (temp_ultra < ultra_measured){
    speed_ref = speed_ref * (delta_speed_ref+1);}
  
  if (ultra_measured == set_ultra && direction_flag != 0) {
    speed_ref = speed_ref;
  } else if (ultra_measured < set_ultra) {
    backward();
  } else {
    forward();
  }
  
  if (speed_ref > max_speed) {speed_ref = max_speed;}
  if (speed_ref < min_speed) {speed_ref = min_speed;}
  pwm_a += (((speed_ref + rps_a) / 2) - rps_a) * 5;
  pwm_b += (((speed_ref + rps_b) / 2) - rps_b) * 5;

  if (pwm_a > max_pwm) {pwm_a = max_pwm;}
  if (pwm_b > max_pwm) {pwm_b = max_pwm;}
  if (pwm_a < min_pwm) {pwm_a = min_pwm;}
  if (pwm_b < min_pwm) { pwm_b = min_pwm;}

  Serial.print("Distance in CM: ");
  Serial.println(ultra_measured);

  Serial.print(speed_ref);
  Serial.print(" , ");
  Serial.print(pwm_a);
  Serial.print(" , ");
  Serial.print(rps_a);
  Serial.print(" , ");
  Serial.print(hole_a);
  Serial.print(" , ");
  Serial.print(pwm_b);
  Serial.print(" , ");
  Serial.print(rps_b);
  Serial.print(" , ");
  Serial.println(hole_b);

  analogWrite(enA, pwm_a);
  analogWrite(enB, pwm_b);

  temp_ultra = ultra_measured;
  temp_speed_a = speed_a;
  temp_speed_b = speed_b;
  speed_a = 0;
  speed_b = 0;

  if (set_distance != 0){
  if ( distance >= set_distance ) {
    stopped();
  }
  }

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

  setPwmFrequency(5, 1024);
  setPwmFrequency(6, 1024);

  Serial.begin(9600);
  enable_interrupt();

  forward();
  //  backward();
}
void enable_interrupt()
{
  Timer1.initialize(1000000); // set timer for 0.1 sec
  attachInterrupt(0, docount_a, RISING);  // increase speed_a when speed sensor pin goes High
  attachInterrupt(1, docount_b, RISING);  // increase speed_a when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void disable_interrupt()
{
  detachInterrupt(digitalPinToInterrupt(2));  // increase speed_a when speed sensor pin goes High
  detachInterrupt(digitalPinToInterrupt(3));  // increase speed_a when speed sensor pin goes High
  Timer1.detachInterrupt(); // enable the timer
}

void stopped()
{
  pwm_a = min_pwm;
  pwm_b = min_pwm;
  direction_flag = 0;
  //turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void forward()
{
  direction_flag = 1;
  //motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void backward()
{
  direction_flag = 2;
  //motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void forward_left(){
  direction_flag = 3;
  //motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  //motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void forward_right(){
  direction_flag = 4;
  //motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void backward_left(){
  direction_flag = 5;
  //motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  //motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backward_right(){
  direction_flag = 6;
  //motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void setPwmFrequency(int pin, int divisor)
{
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void loop() {

}
