
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>

#include "data_struct.h"

#define parachutePin 9
#define SDPin 6
#define timeFailsafe 300000          //time after liftoff to detonate 
#define stabilisationPeriod 5000    //Time to wait to stabilize
#define liftoffSensitivity 0.5
#define parachuteSensitivity 0.2
#define altitudeSensitivity 1.5     //Change to real value (m)
#define writeInterval 500
#define filename "data.txt"
#define buzz 3

//Sensor declaration

Adafruit_BMP280 bme;
FaBo9Axis fabo_9axis;

Servo parachute;
unsigned long long int timeZero, prevTime = 0; //var to keep time of liftoff

struct Temp {
  double fabo = 0;
  double bme = 0;
} tempCurr;

struct Accel {
  double x = 0;
  double y = 0;
  double z = 0;
} accelInit, accelCurr;     //vars to store initial and current acceleration

struct Orient {
  double p = 0;             //Pitch     (arround x-axis)
  double r = 0;             //Roll      (arround y-axis)
  double a = 0;             //Azimouth  (arround z-axis)
} orientInit, orientCurr;   //vars to store initial and current orientation

double altitudeCurr;
double pressureInit, pressureCurr;
double altitudeMax = 0;
File dataFile;

double write_to_sd(File file, Temp temp, Accel accel, Orient orient, double alt, double _press) {
  //if the file didn't open, print an error:
  if (!file) {
    Serial.println("error opening test.txt");
    return -1;//-1 means error
  }

  Serial.print("Writing to txt...");
  
  file.print(String(alt) + " ");
  file.print(String(temp.fabo) + " " + String(temp.bme) + " ");//poia thermokrasia na balo?
  
  //acceleration
  dataFile.print(accel.x);
  dataFile.print(" ");
  
  dataFile.print(accel.y);
  dataFile.print(" ");

  dataFile.print(accel.z);
  dataFile.print(" ");

  //orient
  /*
  dataFile.print(orient->p);
  dataFile.print(" ");

  dataFile.print(orient->r);
  dataFile.print(" ");

  dataFile.print(orient->a);
  dataFile.print(" ");
  */
  
  dataFile.print(String(alt) + " ");
  dataFile.println(String(_press) + " ");

  //dataFile.println();

  Serial.println("done.");
  return 0;

}

int initialise_SD(int pin) {
  Serial.print("Initializing SD card...");

  if (!SD.begin(pin)) {
    Serial.println("initialization failed!");
    return 0;
  }
  SD.remove(filename);
  Serial.println("initialization done.");
  return 1;
}


void setup() {
  pinMode(buzz, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Waiting for initialization");

  //create and setup instances
  parachute.attach(parachutePin);
  parachute.write(3);
  while (!bme.begin());
  while (!fabo_9axis.begin());
  initialise_SD(SDPin);

  dataFile = SD.open(filename, FILE_WRITE);

  //wait for stabillity
  delay(stabilisationPeriod);

  //get initial values
  getAccel(&accelInit);
  accelCurr = accelInit;

  Serial.print(accelInit.x); Serial.print(" ");
  Serial.print(accelInit.y); Serial.print(" ");
  Serial.print(accelInit.z); Serial.println(" G");

  getOrient(&orientInit);
  orientCurr = orientInit;

  getTemp(&tempCurr);
  Serial.print(tempCurr.fabo); Serial.print(" ");
  Serial.print(tempCurr.bme); Serial.println(" C");

  getPressure(&pressureInit);
  pressureCurr = pressureInit;
  Serial.print(pressureInit); Serial.println(" Pa");

  getAltitude(&altitudeCurr);
  Serial.print(altitudeCurr); Serial.println(" m");

  Serial.println("Initialized");

  analogWrite(buzz, 122);
  delay(1500);
  analogWrite(buzz, 0);
  
  while (!detectLiftoff());  //wait until liftoff and then set timeZero
  Serial.println("Liftoff");
  analogWrite(buzz, 122);
  delay(250);
  analogWrite(buzz, 0);
}

void loop() {
  getAccel(&accelCurr);
  getAltitude(&altitudeCurr);
  if(millis() - prevTime >= writeInterval){
    getTemp(&tempCurr);
    getPressure(&pressureCurr);
    write_to_sd(dataFile, tempCurr, accelCurr, orientCurr, altitudeCurr, pressureCurr);
    prevTime = millis();
  }
  if (time2open()) openPara();
}

bool detectLiftoff() {
  getAccel(&accelCurr);
  if (accelCurr.y > accelInit.y + liftoffSensitivity) {
    timeZero = millis();
    return true;
  }
  return false;
}

bool time2open() {
  if ( (altitudeMax - altitudeCurr  > altitudeSensitivity /* NEGATIVE SPEED */) /*|| (accelCurr.y < accelInit.y - parachuteSensitivity  NEGATIVE ACCELERATION )*/ || (millis() - timeZero > timeFailsafe /* FAILSAFE */) ) return true;
  return false;
}

void openPara() {
  Serial.println("Parachute");
  parachute.write(90);
  Serial.println(altitudeMax);
  //write_to_sd(dataFile,&tempCurr,&accelCurr,&orientCurr,altitudeCurr,pressureCurr );
  dataFile.close();
  while (true);
}

void getOrient(Orient *O) {
  O->p = 0; //sensor reading
  O->r = 0;
  O->a = 0;
}

void getAccel(Accel *A) {
  float ax, ay, az;
  fabo_9axis.readAccelXYZ(&ax, &ay, &az);

  A->x = ax; //sensor reading
  A->y = ay;
  A->z = az;
}

void getTemp(Temp *T) {
  float faboT;
  fabo_9axis.readTemperature(&faboT);
  T->fabo = faboT;

  T->bme = bme.readTemperature();
}

void getPressure(double *P) {
  *P = bme.readPressure() / 100;
}

void getAltitude(double *A) {
  *A = bme.readAltitude(pressureInit);
  if (*A > altitudeMax) altitudeMax = *A;
  //Serial.print(altitudeMax);Serial.print(" ");Serial.println(*A);
}
