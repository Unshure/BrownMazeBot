#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h> // USED Encoder library included with Tinsyduino. Documentation:  https://www.pjrc.com/teensy/td_libs_Encoder.html

  /* Measurements:
      Wheel Diameter: 40.15mm
      Circumference: 12.56637cm w/ D = 4cm
      Circumference: 12.61349 exact
      Approximate Gear ratio: 30:1
      Exact Gear ratio: 29.86:1     Meaning everytime the wheel rotates once, the extended shaft rotates 29.86 times
  */

// Constants
#define CIRCUMFERENCE 12.61349    // Wheel circumference
#define MotorGearRatio 29.86      // Exact motor gear ratio

// Right motor
#define RightIN1 20   // AIN1
#define RightIN2 21   // AIN2
#define RightPWM 5

// Left motor
#define LeftIN1 18    // BIN1
#define LeftIN2 17    // BIN2
#define LeftPWM 6

//Standby Pin
#define STBY 19       // Standby Pin, Low = Standby. Documentation https://www.pololu.com/file/0J86/TB6612FNG.pdf

// Signal for ultrasonic sensor
#define Echo 14
#define Trig 4
 unsigned long echo = 0;

// Encoders           //Encoder declarations, eventually move it into a struct and hopefully a class on it's own.
// - Right wheel
#define RightA 0
#define RightB 1
// - Left wheel
#define LeftA 2
#define LeftB 3

// Sensor pin assignments  - Changed rear left & front left pin assignments
#define rearLeftSensor   23
#define rearRightSensor  22

//Front Left sensor
#define frontRightSensor 15
//Front right sensor
#define frontLeftSensor  16

int Spd = 75; // Sets speed to drive robot straight

//Encoder Objects
Encoder encRight(RightA, RightB);
Encoder encLeft(LeftA, LeftB);

//PID
float prevPID;
float total;

float prevEnc;
static float Kp = 0.3, Ki = .0075, Kd = .0075;    //PID constants using encoders to drive straight
static float error, P, I,  D;      // error variables
float total_Enc; //Keeps track of total PID Error using encdoers

//IR Centering
float prevIR;
static float KpIR = .015, KiIR = 0, KdIR = 0;   //PID constants using IR to drive straight
static float errorIR, P_IR, I_IR, D_IR;      // error variables
float total_IR; //Keeps track of total PID error using IR

//Angle Correction
float prevA;
static float KpA = .1, KiA = 0, KdA = 0;   //PID constants using IR to drive straight
static float errorA, P_A, I_A, D_A;      // error variables
float total_A; //Keeps track of total PID error using IR

//Encoders
int encR, encL;

// Need to account for Standby somehow, probably a method.
struct Motor
{
  int IN1;      // Enable 1 for motor
  int IN2;      // Enable 2 for motor
  int PWM;      // PWM input for motor
};
Motor Left;
Motor Right;

///Ultrasonic Code
float pingtime = 0;
float pingShoot = 0;
float sonicDist = 0;
int sonicState = 0;
void ping()
{
  if((millis()-pingShoot) > 100){
    if(sonicState == 0){
      pingtime = millis();
      sonicState = 1;
    }
    if((millis() - pingtime)<=2 && sonicState == 1){
    //Serial.println("Sonic 0");
    digitalWrite(Trig,LOW);
    sonicState = 2;
    }
    else if((millis()-pingtime)>2 && (millis()-pingtime)<5 && sonicState == 2){
    //Serial.println("Sonic 1");
    digitalWrite(Trig,HIGH);
    sonicState = 3;
    }
    else if((millis()- pingtime)>5 || sonicState == 3){
    //Serial.println("Sonic 2");
    digitalWrite(Trig,LOW);
    echo = pulseIn(Echo,HIGH);
    sonicDist = (echo/58.138);
    sonicState = 0;
    pingtime = millis();
    pingShoot = millis();
    }
  }
}

void resetEncoders()
{
  encRight.write(0);
  encLeft.write(0);
}

#define CAPACITY 50       // Order of the Moving Avg filter
                          // The higher the order the longer it takes to change output. Helps smooth erratic sensor inputs
struct SensorV2
{
  int prevValues[CAPACITY];     // A circular buffer that has the CAPACITY previous values of Input Sensor
  int sum;                      // Holds the sum of the previous values of the Input
  int avg;                      // Avg of the values in the buffer | Output of Moving Avg Filter
};

struct UltraSensor
{
  float prevValues[CAPACITY];     // A circular buffer that has the CAPACITY previous values of Input Sensor
  float sum;                      // Holds the sum of the previous values of the Input
  float avg;                      // Avg of the values in the buffer | Output of Moving Avg Filter
};

SensorV2 rearLeft;
SensorV2 frontLeft;
SensorV2 rearRight;
SensorV2 frontRight;
/* Implements the Moving Avg Filter
 *  Because we have the sum of the buffer in a variable
 *  we can update the avg when a new Input
*/
int updateAvg(struct SensorV2 *sensor, int current, int counter)
{
  int temp = sensor->prevValues[counter % CAPACITY]; // Value in buffer that will be replaced
  sensor->prevValues[counter % CAPACITY] = current;
  sensor->sum += -temp + current;
  return sensor->avg = sensor->sum / CAPACITY;
}
// overloading updateAvg to work with avg
/*int updateAvg(struct UltraSensor *sensor, float current, float counter)
{
  float temp = sensor->prevValues[counter % CAPACITY]; // Value in buffer that will be replaced
  sensor->prevValues[counter % CAPACITY] = current;
  sensor->sum += -temp + current;
  return sensor->avg = sensor->sum / CAPACITY;
}
*/
//IRCode/////////////////////////////////////////////////////
int distRearLeft;
int distRearRight;
int distFrontLeft;
int distFrontRight;

int counter = 0;
double BLa = 0.0001;
double BLb = -0.0695;
double BLc = 16.4856;

void readIR(){
  distRearLeft = analogRead(rearLeftSensor); // rear left sensor
  distRearRight = analogRead(rearRightSensor); // rear right sensor
  distFrontLeft = analogRead(frontLeftSensor); // front left sensor
  distFrontRight = analogRead(frontRightSensor); // front right sensor
}

int rLeftFiltered;
int rRightFiltered;
int fLeftFiltered;
int fRightFiltered;

void filterIR(){
  rLeftFiltered = updateAvg(&rearLeft, distRearLeft, counter);
  rRightFiltered = updateAvg(&rearRight, distRearRight, counter);
  fRightFiltered = updateAvg(&frontRight, distFrontRight, counter);
  fLeftFiltered = updateAvg(&frontLeft,distFrontLeft, counter);
  counter++;
}

void SetPinMode(struct Motor *motor, int in1, int in2, int Pwm) // Becareful with the Capitalization of Set
{
  //Motors
  motor->IN1 = in1;
  motor->IN2 = in2;
  motor->PWM = Pwm;
  pinMode(motor->IN1, OUTPUT);
  pinMode(motor->IN2, OUTPUT);
  pinMode(motor->PWM, OUTPUT);
}

// IN1(High) & IN2(Low) = Clockwise
// IN1(Low) IN2(High) = CounterClockwise

// Left Clockwise & Right CounterClockwise = Backwards
// Right Clockwise & Left CounterClockwise = Forwards

void setDirection(bool dirL, bool dirR)
{
  // boolean direc sets direction, if true forwards, false -> backwards
  // Set direction to backwards
  if (dirL == false)
  {
    digitalWrite(Left.IN2, LOW);
    digitalWrite(Left.IN1, HIGH);
  }
  else
  {
    digitalWrite(Left.IN1, LOW);
    digitalWrite(Left.IN2, HIGH);
  }
  if(dirR == false){
    digitalWrite(Right.IN2, HIGH);
    digitalWrite(Right.IN1, LOW);
  }
  else{
    digitalWrite(Right.IN1, HIGH);
    digitalWrite(Right.IN2, LOW);
  }
}
// Set all enables = low to stop motor
// Need to stop before changing direction
// If we don't stop we can drift

void stopMotor()
{
  digitalWrite(Left.IN1, LOW);
  digitalWrite(Left.IN2, LOW);
  digitalWrite(Right.IN2, LOW);
  digitalWrite(Right.IN1, LOW);
}

//Sets the speed of motors individually
void setSpeed(int lSpd, int rSpd)
{
  analogWrite(Left.PWM, lSpd);
  analogWrite(Right.PWM, rSpd);
}

int pidSampleRate = 40;
void driveStraight()
{

  stopMotor(); //Stops motors
  setDirection(true, true);  //Sets direction of Motors to run forward

  if((millis()-prevPID)>=pidSampleRate){

    //Drive Straight
    error = encL - encR; //Using number of ticks as error
    P = error; //Proprotional Error
    I = I + (error); //Integral/Accumulated Error
    D = error - prevEnc;
    prevEnc = error;
    total_Enc = P * Kp + I * Ki + D * Kd;

    //Center between walls
    errorIR =  (fRightFiltered - fLeftFiltered) + (rRightFiltered - rLeftFiltered);
    P_IR = errorIR;
    I_IR = I_IR + (errorIR);
    D_IR = errorIR - prevIR;
    prevIR = errorIR;
    total_IR = P_IR * KpIR + I_IR * KiIR + D_IR * KdIR;

    //Angle Correction
    errorA = ((fLeftFiltered - rLeftFiltered) - (fRightFiltered - rRightFiltered))*-1;
    P_A = errorA;
    I_A = I_A + (errorA);
    D_A = errorA - prevA;
    prevA = errorA;
    total_A = P_A * KpA + I_A * KiA + D_A * KdA;

    total = constrain((int)(total_Enc) + Spd,0,255);

    //Sets speed of motors based on total error
    //setSpeedRight(right, Spd - total);  -- This PID should adjuct proportional to the right wheel
    setSpeed(Spd, total);
    prevPID = millis();
  }
}
void driveBtwnWalls(){
  setDirection(true, true);  //Sets direction of Motors to run forward

  if((millis()-prevPID)>=pidSampleRate){

    //Drive Straight
    error = encL - encR; //Using number of ticks as error
    P = error; //Proprotional Error
    I = I + (error); //Integral/Accumulated Error
    D = error - prevEnc;
    prevEnc = error;
    total_Enc = P * Kp + I * Ki + D * Kd;

    //Center between walls
    errorIR =  (fRightFiltered - fLeftFiltered) + (rRightFiltered - rLeftFiltered);
    P_IR = errorIR;
    I_IR = I_IR + (errorIR);
    D_IR = errorIR - prevIR;
    prevIR = errorIR;
    total_IR = P_IR * KpIR + I_IR * KiIR + D_IR * KdIR;

    //Angle Correction
    errorA = ((fLeftFiltered - rLeftFiltered) - (fRightFiltered - rRightFiltered))*-1;
    P_A = errorA;
    I_A = I_A + (errorA);
    D_A = errorA - prevA;
    prevA = errorA;
    total_A = P_A * KpA + I_A * KiA + D_A * KdA;

    total = constrain((int)(total_IR + total_A) + Spd,0,255);

    //Sets speed of motors based on total error
    //setSpeedRight(right, Spd - total);  -- This PID should adjuct proportional to the right wheel
    setSpeed(Spd, total);
    prevPID = millis();
  }
}

void turn(bool dir, int degree)
{
    stopMotor();
    resetEncoders();
    long ticksPerRotationLeft;
    long ticksPerRotationRight;

    if(degree == 90){
    int ticksPerRotationLeft = 310; //Number of encoder ticks in a 90 degree rotation
    int ticksPerRotationRight = -270;
    }
    else{
      ticksPerRotationLeft = 620;
      ticksPerRotationRight = -540;
    }
    encR = encLeft.read();
    encL = encRight.read();
    if(dir){
      setDirection(true,false);
    }
    else{
      setDirection(false,true);
    }
    setSpeed(Spd,Spd);
    while(encL > ticksPerRotationLeft && encR > ticksPerRotationRight )
    {
      encL = encLeft.read();
      encR = encRight.read();
      //For debugging
      Serial.println(encLeft.read());
      Serial.println(encRight.read());
    }
    stopMotor();
    resetEncoders();
}

int motorSpeed(int prev, int current)
{
  return (CIRCUMFERENCE / MotorGearRatio) / (current - prev) * 1000;
}

void setup()
{
  // intiallizing Motors
  SetPinMode(&Left, LeftIN1 , LeftIN2, LeftPWM);
  SetPinMode(&Right, RightIN1, RightIN2, RightPWM);
  //Standby
  pinMode(STBY, OUTPUT);
  //IR
  pinMode(Echo,INPUT);
  pinMode(Trig,OUTPUT);

  //IRSensor
  pinMode(rearLeftSensor,INPUT);
  pinMode(frontLeftSensor,INPUT);
  pinMode(rearRightSensor, INPUT);
  pinMode(frontRightSensor,INPUT);

  digitalWrite(STBY, HIGH); // Third enable for Motors, STANDBY = LOW -> motor don't run

  Serial.begin(9600);
  Serial.println("This thing does work");
  delay(1000);
}

double encodersToDistance()
{
  //Constants
  double ticksPerRotationRight = -615; //Ticks to rotate forward
  double ticksPerRotationLeft = 615; //Ticks to rotate forward
  double wheelDiameter = 40.15; //40.15mm

  //Calculations
  double circumference = 3.14152965*wheelDiameter;

  double distanceR = abs(encR*circumference/ticksPerRotationRight);
  double distanceL = abs(encL*circumference/ticksPerRotationLeft);

  double distance = (distanceR + distanceL)/2;


  return distance;

}

void loop()
{
  /*READ ENCODERS*/
  //Get the new values of the encoders
  encR = encRight.read()*-1;
  encL = encLeft.read();
  /*DRIVE IN STRAIGHT LINE*/
  //Drives robot straight at speed Spd
  if((encodersToDistance()/10) < 18){
    driveStraight();
  }else{
    setSpeed(0,0);
  }
  if((encodersToDistance()/10) >17 && (encodersToDistance()/10) <18 && ((int)encodersToDistance()%2) ==0){
    Spd -= 10;
  }
  readIR();
  filterIR();
  ping();
  debug();
  serial();
}

void debug(){
  Serial.print("Current time is:   ");
  Serial.print(millis());
  Serial.print("Right Ticks: ");
  Serial.print(encR);
  Serial.print("Left Ticks: ");
  Serial.print(encL);
  Serial.print("THis is the error:  ");
  Serial.print((int)(Spd+total));
  Serial.print("Sonic Value:  ");
  Serial.print(sonicDist);
  Serial.print(" Rear left: ");
  Serial.print(rLeftFiltered);
  Serial.print(" Rear right: ");
  Serial.print(rRightFiltered);
  Serial.print(" front left: ");
  Serial.print(fLeftFiltered);
  Serial.print(" front right: ");
  Serial.print(fRightFiltered);
  Serial.print("IRERROR:   ");
  Serial.print(total_A);
  Serial.print("Dist Trav:  ");
  Serial.println(encodersToDistance()/10);
}
