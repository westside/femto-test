#define FIMU_ACCGYRO_ADDR MPU60X0_DEFAULT_ADDRESS

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

#include <HMC58X3.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>

#include <Wire.h>
#include <SPI.h>

// Adafruit nRF8001 Library
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 7     // This should be an interrupt pin, on Uno thats #2 or #3. IMUduino uses D7
#define ADAFRUITBLE_RST 9

// bluetooth
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
aci_evt_opcode_t status = laststatus;


float ypr[3];
char chrData[17]; // Yaw (5 bytes), Pitch (5 bytes), Roll (5 bytes) ...delimeter is a pipe '|'
char sendbuffersize;


volatile float q0, q1, q2, q3;
volatile float twoKi, twoKp;
float exInt, eyInt, ezInt;
float integralFBx,  integralFBy, integralFBz;
float sampleFreq;
unsigned long lastUpdate, now;


MPU60X0 accgyro;
HMC58X3 magn;
MS561101BA baro;

int16_t gyro_off_x, gyro_off_y, gyro_off_z;
int16_t acc_off_x, acc_off_y, acc_off_z, magn_off_x, magn_off_y, magn_off_z; // for calibration
float acc_scale_x, acc_scale_y, acc_scale_z, magn_scale_x, magn_scale_y, magn_scale_z;

void setup() {

  Serial.begin(115200);

  while (!Serial); // Comment this out if you don't want to open up the Serial Monitor to start initialization

  Serial.println("start");
  Wire.begin();
//  Wire.setClock(400000);
  // MPU 6050
  accgyro = MPU60X0(false, MPU60X0_DEFAULT_ADDRESS);
  accgyro.initialize();
  accgyro.setI2CMasterModeEnabled(0);
  accgyro.setI2CBypassEnabled(1);
  accgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000);
  delay(5);

  TWBR = ((F_CPU / 400000L) - 16) / 2;
  delay(100);



  // HMC58X3()
  magn = HMC58X3();
  magn.init(false); // Don't set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  magn.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
  magn.setMode(0);
  delay(10);
  magn.setDOR(B110);

  acc_scale_x = 1;
  acc_scale_y = 1;
  acc_scale_z = 1;

  magn_scale_x = 1;
  magn_scale_y = 1;
  magn_scale_z = 1;

  acc_off_x = 0;
  acc_off_y = 0;
  acc_off_z = 0;

  magn_off_x = 0;
  magn_off_y = 0;
  magn_off_z = 0;

  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;

  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
  twoKp = twoKpDef;
  twoKi = twoKiDef;
  integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  lastUpdate = 0;
  now = 0;


  zeroGyro();

  // barometer
  baro = MS561101BA();
  baro.init(MS561101BA_ADDR_CSB_LOW);


  // bluetooth
  BTLEserial.begin();
}


void loop() {
  
//  getBarometer();

  btleLoop();
  if (status == ACI_EVT_CONNECTED) {
    float q[4]; // quaternion
    getQ(q);

    btleWriteQuaternian(q[0], q[1], q[2], q[3]);
    
//    arrayPrintln(q, 4);
  }
  delay(50);

  
//  float q[4]; // quaternion
//  getQ(q);
//  arrayPrintln(q, 4);
//
//  delay(20);
  
//
//  
//  float ypr[3];
//  getYawPitchRoll(ypr);
//
//  Serial.print(ypr[0]); Serial.print(" ");
//  Serial.print(ypr[1]); Serial.print(" ");
//  Serial.println(ypr[2]);
//
//  delay(20);
}

void getBarometer() {
    float temperature = NULL, pression = NULL;
  Serial.print("temp: ");
    temperature = baro.getTemperature(MS561101BA_OSR_4096);
  Serial.print(temperature);
  
  Serial.print(" degC pres: ");
    pression = baro.getPressure(MS561101BA_OSR_4096);
  Serial.print(pression);
  Serial.println(" mbar");

  delay(1000);
}

void arrayPrintln(float * arr, int arr_size) {
  for (int i = 0 ; i < arr_size ; i++) {
    Serial.print(arr[i]); Serial.print(" ");
  }
  Serial.println();
}

void getQ(float * q) {
  float val[9];
  getValues(val);
  
  now = micros();
  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  lastUpdate = now;
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  AHRSupdate(val[3] * M_PI / 180, val[4] * M_PI / 180, val[5] * M_PI / 180, val[0], val[1], val[2], val[6], val[7], val[8]);

  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

void getYawPitchRoll(float * ypr) {
  float q[4]; // quaternion
  getQ(q);
//
//  float q0 = q[0];
//  float q1 = q[1];
//  float q2 = q[2];
//  float q3 = q[3];
//
//  float yaw = atan( (2 * (q0 * q3 + q1 * q2)) / (1 - 2 * (q2 * q2 + q3 * q3))  );
//  float pitch = asin(2 * (q0 * q2 - q3 * q1));
//  float roll = atan( 2 * (q0 * q1  + q2 * q3)  / (1 - 2 * (q1 * q1 + q2 * q2)) );
//
//  ypr[0] = yaw;
//  ypr[1] = pitch;
//  ypr[2] = roll;


  //  float gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  //  float gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  //  float gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  //
  //  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  //  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  //  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));

  arr3_rad_to_deg(ypr);
}

void arr3_rad_to_deg(float * arr) {
  arr[0] *= 180/M_PI;
  arr[1] *= 180/M_PI;
  arr[2] *= 180/M_PI;
}

void getValues(float * values) {
  int16_t accgyroval[6];
  accgyro.getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);

  // remove offsets from the gyroscope
  accgyroval[3] = accgyroval[3] - gyro_off_x;
  accgyroval[4] = accgyroval[4] - gyro_off_y;
  accgyroval[5] = accgyroval[5] - gyro_off_z;

  for (int i = 0; i < 6; i++) {
    if (i < 3) {
      values[i] = (float) accgyroval[i];
    }
    else {
      values[i] = ((float) accgyroval[i]) / 16.4f; // NOTE: this depends on the sensitivity chosen
    }
  }


  magn.getValues(&values[6]);
  // calibration
  //    #warning Magnetometer calibration active: have you calibrated your device?
  values[6] = (values[6] - magn_off_x) / magn_scale_x;
  values[7] = (values[7] - magn_off_y) / magn_scale_y;
  values[8] = (values[8] - magn_off_z) / magn_scale_z;
}

void zeroGyro() {
  const int totSamples = 3;
  int raw[11];
  float tmpOffsets[] = {0, 0, 0};

  for (int i = 0; i < totSamples; i++) {
    getRawValues(raw);
    tmpOffsets[0] += raw[3];
    tmpOffsets[1] += raw[4];
    tmpOffsets[2] += raw[5];
  }

  gyro_off_x = tmpOffsets[0] / totSamples;
  gyro_off_y = tmpOffsets[1] / totSamples;
  gyro_off_z = tmpOffsets[2] / totSamples;
}

void getRawValues(int * raw_values) {
  accgyro.getMotion6(&raw_values[0], &raw_values[1], &raw_values[2], &raw_values[3], &raw_values[4], &raw_values[5]);
  magn.getValues(&raw_values[6], &raw_values[7], &raw_values[8]);
}


void  AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
  if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
    float hx, hy, bx, bz;
    float halfwx, halfwy, halfwz;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of magnetic field
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (my * halfwz - mz * halfwy);
    halfey = (mz * halfwx - mx * halfwz);
    halfez = (mx * halfwy - my * halfwx);
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if ((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
    float halfvx, halfvy, halfvz;

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }

  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


/**
 * Fast inverse square root implementation
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}





void btleLoop() {
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println("* Advertising started");
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println("* Connected!");
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); 
      Serial.print(BTLEserial.available()); 
      Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }
  }
}


void btleWriteYPR(float Y, float P, float R) {
  dtostrf(Y, 1, 1, &chrData[0]);
  dtostrf(P, 1, 1, &chrData[6]);
  dtostrf(R, 1, 1, &chrData[11]);
  chrData[5] = '|';
  chrData[10] = '|';
  sendbuffersize = min(20, sizeof(chrData));
  
  BTLEserial.write((byte*)chrData, sendbuffersize);
}


char chrQData[23];

void btleWriteQuaternian(float q1, float q2, float q3, float q4) {
  dtostrf(q1, 1, 1, &chrQData[0]);
  dtostrf(q2, 1, 1, &chrQData[6]);
  dtostrf(q3, 1, 1, &chrQData[12]);
  dtostrf(q4, 1, 1, &chrQData[18]);
  chrQData[5] = ',';
  chrQData[11] = ',';
  chrQData[17] = ',';
  sendbuffersize = min(30, sizeof(chrQData));
  
  BTLEserial.write((byte*)chrQData, sendbuffersize);
}

byte * float2str(float arg) {
  // get access to the float as a byte-array:
  byte * data = (byte *) &arg;
  return data;
}

