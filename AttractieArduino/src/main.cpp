#include <Arduino.h>
#include <AccelStepper.h>

int power;
bool start = false;
bool reverse = false;
char receiveVal;

int ldrPin = A0;

String SerieleCom = "";
bool startCommunication = false;

const char startChar = '#';
const char endChar = '%';

//Motor van de basisplaat
#define motor1Pin1 13
#define motor1Pin2 12
#define motor1Pin3 11
#define motor1Pin4 10

//Motor van karretje 1
#define motor2Pin1 9
#define motor2Pin2 8
#define motor2Pin3 7
#define motor2Pin4 6

//Motor van karretje 2
#define motor3Pin1 5
#define motor3Pin2 4
#define motor3Pin3 3
#define motor3Pin4 2

AccelStepper stepperPlate(AccelStepper::HALF4WIRE, motor1Pin1, motor1Pin3, motor1Pin2, motor1Pin4);
AccelStepper stepperCar1(AccelStepper::HALF4WIRE, motor2Pin1, motor2Pin3, motor2Pin2, motor2Pin4);
AccelStepper stepperCar2(AccelStepper::HALF4WIRE, motor3Pin1, motor3Pin3, motor3Pin2, motor3Pin4);

/*unsigned long startMillis;
unsigned long currentMillis;*/

int stepsPerRevolution = 64; //stappen per omwenteling
float degreePerRevolution = 5.625; //graden per omwenteling

//Patroon van graden die de stappen motoren af zullen gaan
int stepsPlate[] = {70, 0};
int stepsPlateCount = 2;
int stepsPlateIndex = 0;
int stepsCar1[] = {-360, -270, -180, -90, 0};
int stepsCar1Count = 5;
int stepsCar1Index = 0;
int stepsCar2[] = {0, -90, -180, -270, -360};
int stepsCar2Count = 5;
int stepsCar2Index = 0;

//Stappen berekenen uit de graden
float degToSteps(float deg){
  return (stepsPerRevolution / degreePerRevolution) * deg;
}

int readPowerInPercent() {
  int ldr = analogRead(ldrPin);
  ldr = map(ldr, 0, 1023, 100, 0);
  //ldr = 100 - ldr;
  return ldr;
}

void configureCarMotors(double maxSpeed, double acceleration, double speed) {
  stepperCar1.setMaxSpeed(maxSpeed);
  stepperCar1.setAcceleration(acceleration);
  stepperCar1.setSpeed(speed);

  stepperCar2.setMaxSpeed(maxSpeed);
  stepperCar2.setAcceleration(acceleration);
  stepperCar2.setSpeed(speed);
}

void configurePlateMotor() {
  stepperPlate.setMaxSpeed(200);
  stepperPlate.setAcceleration(500);
  stepperPlate.setSpeed(200);
}

void resetMotorPositions() {
  stepperPlate.setCurrentPosition(0);
  stepsPlateIndex = 0;
  stepperCar1.setCurrentPosition(0);
  stepsCar1Index = 0;
  stepperCar2.setCurrentPosition(0);
  stepsCar2Index = 0;

  //De configuratie moet opnieuw uitgevoerd worden wanneer de stappenmotoren opnieuw gepositioneerd worden
  configureCarMotors(1000.0, 900.0, 1000);
  configurePlateMotor();
}

void moveMotors() {
  //Bepaal de volgende positie wanneer er geen afstand naar de volgende positie meer te gaan is
  if(!stepperCar1.isRunning() && stepperCar1.distanceToGo() == 0) {
    stepperCar1.moveTo(degToSteps(stepsCar1[stepsCar1Index]));
    
    if (reverse) {
      stepsCar1Index--;
      if(stepsCar1Index < 0){
        stepsCar1Index = 5;
      }
    }
    else {
      stepsCar1Index++;
      if(stepsCar1Index > stepsCar1Count){
        stepsCar1Index = 0;
      }
    }
  }
  //Zet de motor aan
  stepperCar1.run();

  if(!stepperCar2.isRunning() && stepperCar2.distanceToGo() == 0) {
    stepperCar2.moveTo(degToSteps(stepsCar2[stepsCar2Index]));
    
    if (reverse) {
      stepsCar2Index--;
      if(stepsCar2Index < 0){
        stepsCar2Index = 5;
      }
    }
    else {
      stepsCar2Index++;
      if(stepsCar2Index > stepsCar1Count){
        stepsCar2Index = 0;
      }
    }
  }
  stepperCar2.run();

  if(!stepperPlate.isRunning() && stepperPlate.distanceToGo() == 0) {
    stepperPlate.moveTo(degToSteps(stepsPlate[stepsPlateIndex]));
    stepsPlateIndex++;
    if(stepsPlateIndex > stepsPlateCount){
      stepsPlateIndex = 0;
    }
  }
  stepperPlate.run();
}

void runAttractionCommand(String command) {
String ackStatus = "ACK";

  //Snelheids instellingen
  if(command.equals("speed=0")) {
    configureCarMotors(500.0, 900.0, 500);
  }
  else if(command.equals("speed=1")) {
    configureCarMotors(750.0, 900.0, 750);
  }
  else if(command.equals("speed=2")) {
    configureCarMotors(1000.0, 900.0, 1000);
  }
  else if(command.equals("speed=3")) {
    configureCarMotors(1250.0, 900.0, 1250);
  }
  else if(command.equals("speed=4")) {
    configureCarMotors(1500.0, 900.0, 1500);
  }

  //Reverse commando
  else if(command.startsWith("reverse=")) {
    int index = command.indexOf('=');
    String letter = command.substring(index+1);
    letter.trim();
    if (letter.equals("f")) {
      reverse = false;
    }
    else if(letter.equals("t")) {
      reverse = true;
    }
  } 

  //Geef de ldr waarde door
  else if (command.equals("send")) {
    power = readPowerInPercent();
    Serial.println(readPowerInPercent());
  }

  else if(command.equals("power=")) {
     int index = command.indexOf('=');
     String letter = command.substring(index+1);
     letter.trim();
     if (letter.equals("f")) {
       start = false;
        resetMotorPositions();
     }
     else if(letter.equals("t")) {
       start = true;
     }  
  }

  //Stuur de acknowledgement status
  else{
    String ackStatus = "NACK";
  }
  Serial.println(ackStatus);
}

void setup() {
  Serial.begin(9600);

  //Stap motor configuratie
  configureCarMotors(1000.0, 900.0, 1000);
  configurePlateMotor();
}

void loop() {
  

  if(Serial.available() > 0)  
  {           
    char readByte = Serial.read(); 
    //Start de attractie
    if(readByte == startChar)   {
      SerieleCom = "";
      startCommunication = true;
    }
    else if (startCommunication)
    {
      if(readByte == endChar){
        startCommunication = false;
        runAttractionCommand(SerieleCom);
      }
      else
      {
        SerieleCom += readByte;
      }
    }
    //runAttractionCommand(receiveVal);
  }   

  if (start) {
    moveMotors();
  }
}