#include "I2Cdev.h"
#include "MPU6050.h"

#define TIME_OUT 10

MPU6050 accgyro;
unsigned long int t1;
unsigned int counter;

void setup() {
    Serial.begin(9600);
    accgyro.initialize();
    accgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    accgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    counter = 0;
    accgyro.CalibrateAccel(6);
    accgyro.CalibrateGyro(6);
}

void loop() {
    long int t = millis();

    int16_t ax, ay, az, gx, gy, gz;
    counter++;
    accgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print(counter);
    Serial.print(",");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.println(gz);

    delay(30);
}