#include <Wire.h>
#include <GY80.h>
#include <SoftwareSerial.h>

struct FitkneeInfo {
    GY80_single_scaled acc;
    GY80_single_scaled gyro;
};

GY80 sensor = GY80();     // Create GY80 instance
SoftwareSerial BT(8, 9);  // Setup BT transmit & receive pin

void setup()
{
    // initialize serial communication at 115200 bits per second:
    Serial.begin(9600);

    Serial.println("Initial 9 DOF sensor");
    sensor.begin();       //initialize sensors

    Serial.println("Initial 9 BT serial");
    BT.begin(38400);      // 38400 for HC-05
    
    Serial.println("Bluetooh &  9 DOF sensor ready!!");
}

void SensorPrint(GY80_single_scaled acc, GY80_single_scaled gyro) {
    // Accelerometer values
    Serial.print("Acc:");
    Serial.print(acc.x,3);
    Serial.print(',');
    Serial.print(acc.y,3);
    Serial.print(',');
    Serial.print(acc.z,3);
    Serial.print(' ');

    // Gyroscope values
    Serial.print("Gyro:");
    Serial.print(gyro.x,1);
    Serial.print(',');
    Serial.print(gyro.y,1);
    Serial.print(',');
    Serial.println(gyro.z,1);
}

void loop()
{
    char val_input;
    FitkneeInfo info;
    // Get values from acc & gyro sensors
    info.acc = sensor.a_read_scaled();
    info.gyro = sensor.g_read_scaled();

    // Print out acc & gyro values
    SensorPrint(info.acc, info.gyro);

    // Print out BT data to serial
    if (BT.available()) {
        val_input = BT.read();
        Serial.print(val_input);
    }

    // Send serial data to BT
    if (Serial.available()) {
        val_input = Serial.read();
        Serial.println(val_input);
        BT.write(val_input);
    }

    //delay(1);        // delay in between reads for stability
}
