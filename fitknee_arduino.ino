#include <Wire.h>
#include <GY80.h>
#include <RingBuf.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define SERIAL_SPEED 9600
#define BLUETOOTH_SPEED 115200
#define TX_PIN 10
#define RX_PIN 11
#define THRESHOLD 5
#define RINGBUFSIZE 4
#define JBUFFERSIZE (90 * RINGBUFSIZE)

struct FitkneeInfo {
    unsigned long ts;
    GY80_single_scaled acc;
    GY80_single_scaled gyro;
};

GY80 sensor = GY80();     // Create GY80 instance
SoftwareSerial BT(TX_PIN, RX_PIN);  // Setup BT transmit & receive pin

unsigned long time;
unsigned long count;
RingBuf *buf;

void setup()
{
    // Initialize serial communication at 115200 bits per second:
    Serial.begin(9600);

    // Initialize sensors
    Serial.println("Initial 9 DOF sensor");
    sensor.begin();

    // Initialize another serial port for BT communication
    // 38400 for HC-05
    Serial.println("Initial BT serial");
    BT.begin(115200);
    
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

void JsonCreator() {
    StaticJsonBuffer<JBUFFERSIZE> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    int len = buf -> numElements(buf);
    FitkneeInfo *info;

    root["size"] = len;

    JsonArray& data = root.createNestedArray("data");
    for (int i = 0; i < len; i++) {
        info = (FitkneeInfo *)buf -> peek(buf, i);
        // Handle timestamp
        data.add(info->ts);
        // Handle acc data
        data.add(float_with_n_digits(info -> acc.x, 4));
        data.add(float_with_n_digits(info -> acc.y, 4));
        data.add(float_with_n_digits(info -> acc.z, 4));
        // Handle gyro data
        data.add(float_with_n_digits(info -> gyro.x, 4));
        data.add(float_with_n_digits(info -> gyro.y, 4));
        data.add(float_with_n_digits(info -> gyro.z, 4));
    }

    // Send json message to BT module
    //root.prettyPrintTo(Serial);
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

    if (buf == NULL) {
        // Initialize Ring buffer for FitkneeInfo type
        buf = RingBuf_new(sizeof(FitkneeInfo), RINGBUFSIZE);
    }

    buf -> add(buf, &info);
    if (buf->isFull(buf)) {
        // Write data to BT side and reset buffer
        JsonCreator();
        // Delete RingBuf object
        RingBuf_delete(buf);
        buf = NULL;
    }

    // Calculate sampling rate
    float diff = (current - time) / 1000; 
    if (diff > THRESHOLD) {
        SamplingRate(diff);
    }

    // Read BT data and write to serial
    if (BT.available()) {
        Serial.write(BT.read());
    }

    // Read serial data and send to BT
    if (Serial.available()) {
        BT.write(Serial.read());
    }
}
