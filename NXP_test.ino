#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <math.h>

//#define Serial SerialUSB //if using Qwiic micro board
#define PI 3.1416

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

float AX, AY, AZ, GX, GY, GZ, MX, MY, MZ;
float rho, posi;  float posiF, posiOld = 0, rhoF, rhoOld = 0;
float acc_3D;

void displaySensorDetails(void)
{
  sensor_t accel, sensor , mag;
  accelmag.getSensor(&accel, &mag);
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(accel.name);
  Serial.print  ("Driver Ver:   "); Serial.println(accel.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(accel.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(accel.max_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(accel.min_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(accel.resolution, 8); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(mag.name);
  Serial.print  ("Driver Ver:   "); Serial.println(mag.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(mag.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(mag.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(mag.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(mag.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("GRYOSCOPE");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(sensor.sensor_id, HEX);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void)
{
  Serial.begin(9600);

  /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }

//  Serial.println("FXOS8700 Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accelmag.begin(ACCEL_RANGE_2G))
  {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
//  displaySensorDetails();
}

void loop(void)
{
  sensors_event_t aevent, gevent, mevent;

  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);
  gyro.getEvent(&gevent);

  AX = aevent.acceleration.x;
  AY = aevent.acceleration.y;
  AZ = aevent.acceleration.z;

  GX = (180/PI)* gevent.gyro.x;
  GY = (180/PI)* gevent.gyro.y;
  GZ = (180/PI)* gevent.gyro.z;

  MX = mevent.magnetic.x;
  MY = mevent.magnetic.y;
  MZ = mevent.magnetic.z;

  acc_3D = sqrt(sq(AX) + sq(AY) + sq(AZ)); // should be almost 1g for code to work. No linear acceleration


    /* Display the accel results (acceleration is measured in m/s^2) */
  Serial.print(AX); Serial.print(",");
  Serial.print(AY); Serial.print(",");
  Serial.print(AZ); Serial.print(",");


    /* Display the gyro results (angular velocity is measured in dps) */
  Serial.print(GX); Serial.print(",");
  Serial.print(GY); Serial.print(",");
  Serial.print(GZ); Serial.print(",");
  
  /* Display the mag results (mag data is in uTesla) */

  Serial.print(MX); Serial.print(",");
  Serial.print(MY); Serial.print(",");
  Serial.print(MZ); Serial.print(",");
  
//  bool ID[8] = {0,0,0,0,0,0,0,0};
//  if (abs(GX)>WarningValue) { ID[7] = 1; }
//  if (abs(GY)>WarningValue) { ID[6] = 1; } 
//  if (abs(GZ)>WarningValue) { ID[5] = 1; } 
//  if (abs(GX)>ErrorValue) { ID[4] = 1; }
//  if (abs(GY)>ErrorValue) { ID[3] = 1; } 
//  if (abs(GZ)>ErrorValue) { ID[2] = 1; }
//  byte gyroFlag = 0;
//  for (int i=0; i<8; i++) {
//    gyroFlag |= ID[i] << i;
//  }

  Serial.print(micros());  
//  Serial.print(gyroFlag,BIN); 
  Serial.println("");

  delay(100);
}
