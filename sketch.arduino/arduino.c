#include "I2Cdev.h"
#include "MPU6050.h"

#define TIME_OUT 10

MPU6050 accgyro;
unsigned long int t1;

void setup() {
    Serial.begin(9600);
    accgyro.initialize();
    accgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    accgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

void loop() {
    long int t = millis();
    if( t1 < t ){
        int16_t ax, ay, az, gx, gy, gz;

        t1 = t + TIME_OUT;
        accgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        Serial.println("ax");
        Serial.println(ax);
        Serial.println("ay");
        Serial.println(ay);
        Serial.println("az");
        Serial.println(az);

        Serial.println("wx");
        Serial.println(gx);
        Serial.println("wy");
        Serial.println(gy);
        Serial.println("wz");
        Serial.println(gz);
    }
}