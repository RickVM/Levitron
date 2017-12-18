#include <Arduino.h>
#include <PID_v1.h>
#include <NewPing.h>

#define PIN_TRIGGER 3
#define PIN_ECHO 4
#define PIN_INPUT A0
#define PIN_OUTPUT 6
#define MAX_DISTANCE 200

NewPing sonar(PIN_TRIGGER, PIN_ECHO, MAX_DISTANCE);

//Define Variables we'll be connecting to
double Setpoint, Input, Output, gap;
double Sp =180;

unsigned long lastPrint = 0;
unsigned long printTime = 100;


//Define the aggressive and conservative Tuning Parameters
double aggKp=2, aggKi=0.2, aggKd=2;
double consKp=0.5, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

struct information {
  double measured;
  double errorValue;
  double setPoint;
  double P;
  double I;
  double D;
  double temperature;
  double output;
} INFO;


String message;

void gatherDebugData() {
INFO.setPoint = Setpoint;
 INFO.measured = Input;
 INFO.errorValue = gap;
 INFO.output = Output;
 INFO.temperature = 0;
}

void formatMessage(void) {  
  message = "#{";
  message += "\"Mv\":"; message += String(INFO.measured); message += ",";
  message += "\"Ev\":"; message += String(INFO.errorValue); message += ",";
  message += "\"Sp\":"; message += String(INFO.setPoint); message += ",";
  message += "\"P\":"; message += String(INFO.P); message += ",";
  message += "\"I\":"; message += String(INFO.I); message += ",";
  message += "\"D\":"; message += String(INFO.D); message += ",";
  message += "\"Temp\":"; message += String(INFO.temperature); message += ",";
  message += "\"Out\":"; message += String(INFO.output);
  message += "}@";
}

void sendMessage(void) {
  if (message.startsWith("#") && message.endsWith("@"))
  {
    //Send message if formated
    Serial.println(message);
  }
}

void sendDebugData() {
    if((millis() )> lastPrint + printTime) {
    gatherDebugData();
    formatMessage();
    sendMessage();
    lastPrint = millis();
    }
}


void setup()
{
  //initialize the variables we're linked to
  //Input = analogRead(PIN_INPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIGGER, OUTPUT);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
Setpoint = Sp;

  Serial.begin(115200);
    message = "";
}

void loop()
{
  Input = analogRead(PIN_INPUT);
  Input = map(Input, 0, 1024, 255, 0);
  /*double duration;
  digitalWrite(PIN_TRIGGER, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(PIN_TRIGGER, HIGH);
  //delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(PIN_TRIGGER, LOW);
  duration = pulseIn(PIN_ECHO, HIGH);
  Input = (duration / 2) / 29.1;
  Input= Input *28;
  if(Input > 255) {
      Input = 255;
  }*/


  gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
    INFO.P = consKp;
    INFO.I = consKi;
    INFO.D = consKd;
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
    INFO.P = aggKp;
    INFO.I = aggKi;
    INFO.D = aggKd;
  }

  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
  //sendDebugData();
}