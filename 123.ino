
#include <PID_v1.h>
#include "ESP8266WiFi.h"
#include <Wire.h>
#include <Math.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <SoftwareSerial.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define MAX_CLIENTS 1
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
WiFiServer server(23);
WiFiClient serverClients[MAX_CLIENTS];
const char* ssid="Lab330";
const char* password="DJHAN330";
unsigned long previousMillis=0;
const long interval=100;

int t=0;
int LED1 = 13;
int LED2 = 0;
int speedR = 68;
int speedRadd = 0;
int speedL = 50;
int speedLadd = 0;
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
int  Setangle= 0;
int a=5;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double Setpoint, Input, Output;
double k, kp=0.4,ki=7,kd=0.5;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, REVERSE);
void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  analogWrite(LED1, 0 );
  analogWrite(LED2, 0 );
  Serial.begin(115200);
  Wire.begin();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.config(IPAddress(192,168,0,20), // IP
  IPAddress(192,168,0,1), // gateway
  IPAddress(255,255,255,0)); // netmask
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Setpoint=0;

#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;//X方向
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;//Y方向
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  server.begin();
  server.setNoDelay(true);

  //PID
  Setpoint=0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 5);
}

void loop() {
  /* Update all the values */
  unsigned long currentMillis=millis();
  uint8_t i;
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;                                           
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees


  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter

  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter


  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;


  /* Print Data */
   if(server.hasClient()){
    for (i=0;i<MAX_CLIENTS;i++){
      if(!serverClients[i] || !serverClients[i].connected()){
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i]=server.available();
        continue;
      }
    }
    WiFiClient serverClients=server.available();
    serverClients.stop();
  }
  //Input=compAngleX+Setangle;
 
  if(currentMillis-previousMillis>=interval){
    previousMillis=currentMillis;
    t=t+1;
    for(i=0;i<MAX_CLIENTS;i++){
     
        if(serverClients[i]&&serverClients[i].connected()){
          serverClients[i].println(compAngleX);
          Serial.write(serverClients[i].read());
          k= serverClients[i].read();
          if(k>=100 && k<200)
          {
            Setangle=k-150;
          }
          else if(k>=200 && k<300)
          {
              kp=k-200;
              
          }
          else if(k>=300 && k<400)
          {
              ki=k-300;
              
          }
          else if(k >=400 && k<500)
          {
              kd=k-400;
              
          }
          myPID.SetTunings(kp,ki,kd);
          Input=compAngleX-Setangle;
          //Serial.println(Setangle);
          delay(1);
          myPID.Compute();
         if(Input>=0)
            { 
              speedRadd=Output;
              analogWrite(LED1, speedR+speedRadd);
              analogWrite(LED2, speedL); 
            }
         else
            {
              
              speedLadd=Output;
              analogWrite(LED1, speedR );
              analogWrite(LED2, speedL+speedLadd);
            }
           //Serial.println(compAngleX);
        }
        else
          {
            kp=1;
            ki=8;
            kd=1;
            Setangle=0;
            analogWrite(LED1, 0 );
            analogWrite(LED2, 0 );
          }

    }
    
  }




  
  //Serial.print(roll); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  //Serial.print("\t");

 // Serial.print("\r\n");
  delay(2);
}









         
