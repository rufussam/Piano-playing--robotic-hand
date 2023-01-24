//Define servo and stepper motor libraries
#include<Servo.h>
#include <AccelStepper.h>

//Defining interupts 
#define limitLeftPin 3
#define limitRightPin 2
#define ledpin 13

//Defining pins for stepper motor 
#define enablePin 5
#define pulsePin 7
#define dirPin 6

int temp=0;
int led_state = 0;
bool left_limit_flag = false;
bool right_limit_flag = false;

AccelStepper stepper(AccelStepper::FULL2WIRE,dirPin , pulsePin);

//Defining servo motors
Servo motor;
Servo motor1;
Servo motor2;
Servo motor3;

int p=0;
char notes[39] = "CCGGAAGFFEEDDCGGFFEEDGGFFEEDCGGAGFFEEDC"; //Notes of the song
char Octave[8] = "CDEFGABC"; //Notes in an Octave
char f1,f2,f3,f4;
int range_min = 0; //Index finger
int range_max = 3; //pinky finger
int match, j, i;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motor.attach(10); //Index finger
  motor1.attach(11);
  motor2.attach(12);
  motor3.attach(13); //pinky finger
  rest();

  Serial.println("Hand Activated");
  pinMode(enablePin,OUTPUT);
  pinMode(pulsePin,OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(ledpin, OUTPUT);
  pinMode(limitLeftPin, INPUT_PULLUP);
  pinMode(limitRightPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(limitLeftPin), LEFTISR, RISING); //Rising - Detects when the interupt changes from 0 to 1
  attachInterrupt(digitalPinToInterrupt(limitRightPin), RIGHTISR, RISING); //Rising - Detects when the interupt changes from 0 to 1

  digitalWrite(enablePin,HIGH);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);

  home_stepper(); //For calibrating the end of the piano

  //stepper.setMaxSpeed(2000);
  stepper.setMaxSpeed(4000);
  //stepper.setAcceleration(1000);
  stepper.setAcceleration(2000);
  stepper.moveTo(-16500); //moves left
  //Serial.println("here!");
}

//This is essential to be executed before any rail movement
void rest()
{
    motor.write(130);
    delay(100);
    motor1.write(140);
    delay(100);
    motor2.write(150);
    delay(100);
    motor3.write(70);
    delay(100);
  
}

void home_stepper()
{
  stepper.moveTo(50000);
  Serial.println("Move towards the left limit switch");
  
  //Run until we hit the left interupt
  while(!left_limit_flag)
  {
   stepper.run(); 
  } 

  Serial.println("Reached limit switch");
  //Serial.println(left_limit_flag);
  //Once reached set that position as zero
  stepper.setCurrentPosition(0);
  long dtoGo = stepper.distanceToGo();
  Serial.println(dtoGo);

  if(left_limit_flag)
  {
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(200);
    stepper.moveTo(-100);
    
    while(digitalRead(limitLeftPin)==0)
    {
     stepper.run();
    }

    Serial.println("Completed");
    long theCurrentPosition = stepper.currentPosition();
    Serial.println(theCurrentPosition);
    left_limit_flag = false;
    stepper.setCurrentPosition(0);
    Serial.println("Homed");
    Serial.println("-------------------------------------------");
  }
}


void LEFTISR()
{ 
  stepper.setMaxSpeed(0);
  stepper.stop(); //stop the running of stepper motor once the left interupt is reached
  led_state = !led_state;
  digitalWrite(ledpin,led_state);
  left_limit_flag = true;
}

void RIGHTISR()
{
  stepper.setSpeed(0);
  stepper.stop(); //stop the running of stepper motor once the right interupt is reached
  led_state = !led_state;
  digitalWrite(ledpin,led_state);
  right_limit_flag = true;
  //stepper.moveTo(50000);
}
void loop()
{ 
  //Keep running whenever we have distance to be moved
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  } 
  f1 = Octave[0];
  f2 = Octave[1];
  f3 = Octave[2];
  f4 = Octave[3];


for(i=0; i<strlen(notes); i++)
{
  int range=0;
  Serial.print(notes[i]); 
  //Trigger the finger for corresponding notes within the range of the hand
    if(f1==notes[i])
      {
      motor.write(80);
      delay(1000);
      motor.write(130);
      delay(500);
      range =1;
      }
    if(f2==notes[i])
      {
      motor1.write(80);
      delay(1000);
      motor1.write(140);
      delay(500);
      range =1;
      }
    if(f3==notes[i])
      {
      motor2.write(70);
      delay(1000);
      motor2.write(150);
      delay(500);
      range =1;
      }
    if(f4==notes[i])
      {
      motor3.write(25);
      delay(1000);
      motor3.write(70);
      delay(500);
      range =1;
      }
    Serial.println(int(i));
  //If not in range, move to the respective position maintaining the fingers in the rest position
  if(range==0)
  { 
    //Serial.println("printing i");
    //Serial.println("i m here as out of range");
    rest();
  
    for(j=0;j<strlen(Octave);j++)
    {
      //Serial.print("entering");
      //Serial.println(notes[i]);
      //Serial.println(Octave[j]);
      if(notes[i]==Octave[j])
      {
        match = j;
        i=i-1;
        Serial.print(i);
        //Serial.print("inside");
        break;
      }
    }

    if (abs(j-range_min) >abs(j-range_max))
      {
      f1 = Octave[j-3];
      f2 = Octave[j-2];
      f3 = Octave[j-1];
      f4 = Octave[j];
      range_min = j-3;
      range_max =j;
      temp= abs((j+1)-range_max)*2100;
      stepper.setCurrentPosition(0);
      stepper.moveTo(-temp);
      while(stepper.distanceToGo() != 0)
      {
      stepper.run();
      }
      }
    else
      {
      f1 = Octave[j];
      f2 = Octave[j+1];
      f3 = Octave[j+2];
      f4 = Octave[j+3];
      temp=0;
      range_min = j;
      range_max = j+3;
      temp = abs((j-1)-range_min)*2100; 
      stepper.setCurrentPosition(0);
      stepper.moveTo(temp);
      while(stepper.distanceToGo() != 0)
      {
      stepper.run();
      }
      }
  
  }
   
}

  rest();
  exit(1);

}



