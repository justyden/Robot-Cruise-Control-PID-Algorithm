// This program implements a basic cruise control for the robot.

#include "TimerOne.h"
#include "List.hpp"
 
// Constants for Interrupt Pins
// Change values if not using Arduino Uno
 
const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0 (Encoder)
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1 (Encoder)
 
// Integers for pulse counters
unsigned int counter1 = 0;
unsigned int counter2 = 0;
 
// Float for number of slots in encoder disk
float diskslots = 20;  // Change to match value of encoder disk

#define FAST_SPEED 250
#define MID_SPEED 200
#define SLOW_SPEED  150
#define speedPinR 5    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 
#define SERVO_PIN     9  //servo connect to D9
#define Echo_PIN    4
#define Trig_PIN    10
#define BUZZ_PIN     13

List<float> speedData(true);
int checkTime = 0; // Keeps track of the time.
int currentPower = 200;
float sameSpeed = 400;
boolean checked = false;
int printTime = 0;

/*motor control*/
void go_Advance(int inputSpeed)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL, inputSpeed);
  analogWrite(speedPinR, inputSpeed);
}

void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}

void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}

void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}

void stop_Stop()    //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}

/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

// Turns the buzzer on.
void buzz_ON()   //open buzzer
{
  
  for(int i=0;i<100;i++)
  {
   digitalWrite(BUZZ_PIN,LOW);
   delay(2);//wait for 1ms
   digitalWrite(BUZZ_PIN,HIGH);
   delay(2);//wait for 1ms
  }

// Turns the buzzer off.
}
void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
  
}

// Sets the buzzer on quickly then turns it off.
void alarm(){
   buzz_ON();
 
   buzz_OFF();
}

// Interrupt Service Routines
 
// Motor 1 pulse count ISR
void ISR_count1()  
{
  counter1++;  // increment Motor 1 counter value
} 
 
// Motor 2 pulse count ISR
void ISR_count2()  
{
  counter2++;  // increment Motor 2 counter value
} 
 
// This is the function that enables the cruise control. It also adds data to console.
void cruiseControl()
{
  Timer1.detachInterrupt();  // Stop the timer

  go_Advance(currentPower); // Maintain a constant speed.

  float checkSpeed = (counter1 / diskslots) * 240; // This is because the robot is check 4 times in 1s in this case.
  float errorCorrection = checkSpeed / sameSpeed; // Determines the amount of error that needs to be fixed.

  if (printTime >= 6) { // This keeps track of the amount of time data has printed to the list.
    printTime = 0;
  }

  if (checkSpeed > 100 && printTime == 5) { // This means data is ready to be added to the list.
  speedData.add(checkSpeed);
  }

  // This is the algorithm that fixes the speed and keeps it constant.
  if (errorCorrection <= 0.5) { // This makes it increase a lot.
    currentPower = currentPower + 6;
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 0.85) { // This makes it increase slightly.
    currentPower = currentPower + 5;
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 0.9) { // This makes it increase slightly.
    currentPower = currentPower + 4;
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 0.95) { // This is a very minor increase.
    currentPower = currentPower + 3;
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 0.97) { // This is a very minor increase.
    currentPower = currentPower + 2;
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 1.0) { // This keeps the speed the same.
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 1.03) { // This keeps the speed the same.
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 1.05) { // This slightly decreases the speed.
    currentPower = currentPower - 3;
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 1.1) { // This slightly decreases the speed.
    currentPower = currentPower - 4;
    go_Advance(currentPower);
  }
  else if (errorCorrection <= 1.5) { // This makes it decrease a lot.
    currentPower = currentPower - 6;
    go_Advance(currentPower);
  }

  counter1 = 0;  //  reset counter to zero

  counter2 = 0;  //  reset counter to zero
  checkTime = checkTime + 1;
  printTime = printTime + 1;

  Timer1.attachInterrupt( cruiseControl );  // Enable the timer
}

void setup() 
{
  Serial.begin(9600);

  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);  
  
  Timer1.initialize(250000);
  attachInterrupt(digitalPinToInterrupt (MOTOR1), 
     ISR_count1, RISING);  
    // Increase counter 1 when speed sensor pin goes High
  
  attachInterrupt(digitalPinToInterrupt (MOTOR2),
     ISR_count2, RISING); 
   // Increase counter 2 when speed sensor pin goes High
  
   Timer1.attachInterrupt(cruiseControl);
     // Enable the timer  (this supports serial print for testing)
} 
void loop()
{
  go_Advance(200);
  // This prints the data to the console after the robot has gathered the results.
  if (checkTime > 200 && !checked) {
    Serial.print(speedData.getSize());
    Serial.print(" ");
    for (int i = 0; i < speedData.getSize(); ++i) {
      Serial.print(speedData.getValue(i));
      Serial.print(" ");
    }
    checked = true;
  }
}