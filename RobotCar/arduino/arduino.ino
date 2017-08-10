//add above light sensor

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h> 

#define XSHUT_frontSensor_pin1 8
#define XSHUT_aboveSensor_pin2 9

//#define frontSensor_newAddress 41 //not required address change
#define aboveSensor_newAddress 42

//accerate
#define STOP 90
#define FW 99
#define BW 82

//steering
#define CTFW 90
#define CTBW 90
#define LE 135
#define RI 65

VL53L0X frontSensor;
VL53L0X aboveSensor;

Servo steering;
Servo accerate;

const int steeringPin = 6;
const int acceratePin = 7;

int incomingByte = 0;
int ledPin = 13;
char input[11];

char *command;
int degree;
int time;

void setup() {
  Serial.begin(9600);
  setUpLightSensors();
  setUpCarServos();
}

void setUpLightSensors(){
  pinMode(XSHUT_frontSensor_pin1, OUTPUT);
  pinMode(XSHUT_aboveSensor_pin2, OUTPUT);
  Wire.begin();
  
  //change address of a light sensor and power up the next one
   
  pinMode(XSHUT_aboveSensor_pin2, INPUT);
  delay(10);
  aboveSensor.setAddress(aboveSensor_newAddress);
  pinMode(XSHUT_frontSensor_pin1, INPUT);
  delay(10);
  
  
  frontSensor.init();
  aboveSensor.init();
  
  frontSensor.setTimeout(500);
  aboveSensor.setTimeout(500);
  
  frontSensor.startContinuous();
  aboveSensor.startContinuous();
  
//start light sensor
//frontSensor.init(true);
//frontSensor.setTimeout(500);
}

void setUpCarServos(){
  //set servo pins
  steering.attach(steeringPin);
  accerate.attach(acceratePin);
  
  //start car servos
  delay(1000);
  steering.write(CTFW);
  delay(1000);
}

void loop() {
//    Serial.print(frontSensor.readRangeContinuousMillimeters());
//    Serial.print(" : ");
//    Serial.println(aboveSensor.readRangeContinuousMillimeters());
   if (Serial.available() > 0) {
    // read the incoming byte
    int n = Serial.available();
    for (int i = 0; i < n; i++) {
      input[i] = Serial.read();
    }
    input[n] = '\0';
    
    command = strtok(input," ");
    degree = atoi(strtok('\0'," "));
    time = atoi(strtok('\0'," "));

    if(strcmp(command,"FW")==0)forward();
    else if (strcmp(command,"BW")==0) backward();
    else if (strcmp(command,"TL")==0) turnLeft();
    else if (strcmp(command,"TR")==0) turnRight();
    else wait();
    
  }
  delay(1000);
}

boolean checkDistance(){
    long frontDis = (frontSensor.readRangeContinuousMillimeters());
    long aboveDis = (aboveSensor.readRangeContinuousMillimeters());
    Serial.print(frontDis);
    Serial.print("   ");
    Serial.println(aboveDis);
    if(!frontSensor.timeoutOccurred() && !aboveSensor.timeoutOccurred()){
      if(frontDis<=1000 || aboveDis<=220) return false;
      else return true;
    }
    return false;
}

void forward(){
  int i,cnt=0;
  if(time>100){
    cnt = time/100;
  }
  steering.write(CTFW);
  accerate.write(FW);

  for(i=0;i<cnt&&checkDistance();i++)
      delay(100);
  wait();
}

void backward(){
   checkDistance();
   steering.write(CTBW);
   accerate.write(BW);
   delay(time);
   wait();
}

void wait(){
  accerate.write(STOP);
  delay(1000);
}

void turnLeft(){
  steering.write(LE);
  delay(1000);
}

void turnRight(){
  steering.write(RI);
  delay(1400);
}

