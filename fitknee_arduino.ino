#include <Wire.h>
#include <GY80.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define TX_PIN 8
#define RX_PIN 9
#define THRESHOLD 5

struct FitkneeInfo {
    unsigned long ts;
    GY80_single_scaled acc;
    GY80_single_scaled gyro;
};

GY80 sensor = GY80();     // Create GY80 instance
SoftwareSerial BT(RX_PIN, TX_PIN);  // Setup BT transmit & receive pin

unsigned long time;
unsigned long count;

void setup()
{
    // Initialize serial communication at 115200 bits per second:
    Serial.begin(9600);

    // Initialize sensors
    Serial.println("Initial 9 DOF sensor");
    sensor.begin();

    // Initialize another serial port for BT communication
    // 38400 for HC-05
    Serial.println("Initial 9 BT serial");
    BT.begin(38400);
    
    Serial.println("External module ready!!");

    // Initialize global variable
    time = millis();
    count = 0;
}

void SensorPrint(unsigned long ts, GY80_single_scaled acc, GY80_single_scaled gyro) {
    // Accelerometer values
    Serial.print("Timestamp:");
    Serial.print(ts);
    Serial.print(" Acc:");
    Serial.print(acc.x,3);
    Serial.print(',');
    Serial.print(acc.y,3);
    Serial.print(',');
    Serial.print(acc.z,3);

    // Gyroscope values
    Serial.print(" Gyro:");
    Serial.print(gyro.x,1);
    Serial.print(',');
    Serial.print(gyro.y,1);
    Serial.print(',');
    Serial.println(gyro.z,1);
}

void SamplingRate(float diff) {
    // Count sample rate
    float rate = count / diff;
    Serial.print("Sampling count: ");
    Serial.print(count);
    Serial.print(", Sampling rate: ");
    Serial.println(rate, 6);
    time = millis();
    count = 0;
}

void JsonCreator(FitkneeInfo info) {
    StaticJsonBuffer<128> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();

    root["ts"] = info.ts;
    // Handle acc data
    JsonArray& acc = root.createNestedArray("acc");
    acc.add(float_with_n_digits(info.acc.x, 6));
    acc.add(float_with_n_digits(info.acc.y, 6));
    acc.add(float_with_n_digits(info.acc.z, 6));
    // Handle gyro data
    JsonArray& gyro = root.createNestedArray("gyro");
    gyro.add(float_with_n_digits(info.gyro.x, 6));
    gyro.add(float_with_n_digits(info.gyro.y, 6));
    gyro.add(float_with_n_digits(info.gyro.z, 6));

    root.printTo(BT);
}

void loop()
{
    FitkneeInfo info;
    char val_input;
    unsigned long current = millis();

    // Get values from acc & gyro sensors
    info.ts = current;
    info.acc = sensor.a_read_scaled();
    info.gyro = sensor.g_read_scaled();
    count++;

    // Print out acc & gyro values
    //SensorPrint(current, info.acc, info.gyro);

    // Create Json message
    JsonCreator(info);

    // Calculate sampling rate
    float diff = (current - time) / 1000; 
    if (diff > THRESHOLD) {
        SamplingRate(diff);
    }

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
