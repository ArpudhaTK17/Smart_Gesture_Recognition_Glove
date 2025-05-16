#include "I2Cdev.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


uint8_t receiverMac[] = {0xEC, 0x62, 0x60, 0x9B, 0xA2, 0x7C};

typedef struct struct_message {
    char letter;
} struct_message;

struct_message dataToSend;

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
const int flexPin1 = 34; // First flex sensor
const int flexPin2 = 33; // Second flex sensor
const int flexPin3 = 32; 
const int flexPin4 = 35; 
const int flexPin5 = 36; 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



char getLetter(float X, float Y, float Z,int flexValue1, int flexValue2,int flexValue3,int flexValue4) {
    if (flexValue1<30 && flexValue2>40  && flexValue3>40 && flexValue4>70 && Y>45) return 'A';

    else if (flexValue1>42 && flexValue1<70 && flexValue2>5 && flexValue2<25 && flexValue3>-5 && flexValue3<25 && flexValue4>30 && flexValue4<50 && Y>40 && Y<50 && Z<0 && Z<10) return 'B';
    
    else if (flexValue1>35 && flexValue1<50 && flexValue2>50 && flexValue2<65 && flexValue3>40 && flexValue3<60 && flexValue4>55 && flexValue4<80 && Y>25 && Y<39 ) return 'C';
    
    else if (flexValue1>45 && flexValue1<65 && flexValue2>-5 && flexValue2<10 && flexValue3>35 && flexValue3<55 && flexValue4>75 && flexValue4<100 && Y>55 && Y<75 ) return 'D';
    
    else if (flexValue1>70 && flexValue1<80 && flexValue2>70 && flexValue2<80 && flexValue3>65 && flexValue3<70 && flexValue4>75 && flexValue4<80 &&X>-15 && X<-5 && Y>60 && Y<70 && Z>-30 && Z<-26) return 'E';
    
    else if (flexValue1>60 && flexValue1<65 && flexValue2>45 && flexValue2<55 && flexValue3>-10 && flexValue3<0 && flexValue4>-10 && flexValue4<0 &&X>-35 && X<-25 && Y>65 && Y<75 && Z>-15 && Z<-5) return 'F';
    
    else if (flexValue1>35 && flexValue1<40 && flexValue2>10 && flexValue2<15 && flexValue3>70 && flexValue3<75 && flexValue4>100 && flexValue4<115 &&X>-25 && X<-15 && Y>15 && Y<25 && Z>-5 && Z<0) return 'G';
    
    else if (flexValue1>65 && flexValue1<75 && flexValue2>5 && flexValue2<10 && flexValue3>5 && flexValue3<10 && flexValue4>100 && flexValue4<120 &&X>-30 && X<-23 && Y>15 && Y<25 && Z>0 && Z<5) return 'H';
    
    else if (flexValue1>70 && flexValue1<80 && flexValue2>70 && flexValue2<75 && flexValue3>65 && flexValue3<75 && flexValue4>85 && flexValue4<100 &&X>-50 && X<-45 && Y>75 && Y<80 && Z>-10 && Z<0) return 'I';
    
    else if (flexValue1>60 && flexValue1<70 && flexValue2>75 && flexValue2<80 && flexValue3>58 && flexValue3<62 && flexValue4>85 && flexValue4<95 &&X>-85 && X<-79 && Y>43 && Y<48 && Z>-47 && Z<-40) return 'J';
    
    else if (flexValue1>0 && flexValue1<10 && flexValue2>0 && flexValue2<10 && flexValue3>0 && flexValue3<5 && flexValue4>65 && flexValue4<75 &&X>-70 && X<-60 && Y>75 && Y<80 && Z>-10 && Z<0) return 'K';
    
    else if (flexValue1>10 && flexValue1<20 && flexValue2>5 && flexValue2<10 && flexValue3>60 && flexValue3<70 && flexValue4>95 && flexValue4<110 &&X>-55 && X<-45 && Y>65 && Y<70 && Z>0 && Z<5) return 'L';
    
    else if (flexValue1>65 && flexValue1<70 && flexValue2>75 && flexValue2<80 && flexValue3>65 && flexValue3<70 && flexValue4>90 && flexValue4<105 &&X>-42 && X<-35 && Y>55 && Y<60 && Z>-35 && Z<-20) return 'M';
    
    else if (flexValue1>55 && flexValue1<75 && flexValue2>55 && flexValue2<65 && flexValue3>35&& flexValue3<50 && flexValue4>70 && flexValue4<85 &&X>-58 && X<-50 && Y>70 && Y<80 && Z>-5 && Z<5) return 'O';
    
    else if (flexValue1>30 && flexValue1<35 && flexValue2>0 && flexValue2<5 && flexValue3>25 && flexValue3<35 && flexValue4>80 && flexValue4<90 &&X>-20 && X<-15 && Y>5 && Y<10 && Z>0 && Z<5) return 'P';
    
    else if (flexValue1>25 && flexValue1<30 && flexValue2>15 && flexValue2<20 && flexValue3>75 && flexValue3<80 && flexValue4>85 && flexValue4<95 &&X>-62 && X<-55 && Y>-55 && Y<-45 && Z>-5 && Z<5) return 'Q';
    
    else if (flexValue1>45 && flexValue1<50 && flexValue2>-10 && flexValue2<-5 && flexValue3>-5 && flexValue3<5 && flexValue4>80 && flexValue4<85 &&X>-30 && X<-15 && Y>58 && Y<65 && Z>-30 && Z<-20) return 'R';
    
    else if (flexValue1>55 && flexValue1<60 && flexValue2>60 && flexValue2<65 && flexValue3>65 && flexValue3<75 && flexValue4>90 && flexValue4<100 &&X>-85 && X<-80 && Y>75 && Y<80 && Z>50 && Z<55) return 'S';
    else return ' ';  // Default case
}




void sendData() {
    
  int flexValue1,flexValue2,flexValue3,flexValue4,flexValue5;
  flexValue1 = analogRead(flexPin1); // thumb
  flexValue2 = analogRead(flexPin2); // index
  flexValue3 = analogRead(flexPin3); // middle
  flexValue4 = analogRead(flexPin4); // ring
  flexValue5 = analogRead(flexPin5); // little
  //Serial.print("sensor: ");
  flexValue1 = map(flexValue1,3100,3800,0,100);
  flexValue2 = map(flexValue2,3100,4000,0,100);
  flexValue3 = map(flexValue3,3400,4100,0,100);
  flexValue4 = map(flexValue4,2850,2950,0,100);
  flexValue5 = map(flexValue5,3100,3800,0,100);
  Serial.print(" ");
  Serial.print(flexValue1);
  Serial.print(" ");
  Serial.print(flexValue2);
  Serial.print(" ");
  Serial.print(flexValue3);
  Serial.print(" ");
  Serial.print(flexValue4);
  Serial.print(" ");
  Serial.print(flexValue5);
  Serial.print(" ");

    dataToSend.letter = getLetter(ypr[0],ypr[1],ypr[2],flexValue1,flexValue2,flexValue3,flexValue4);

    esp_err_t result = esp_now_send(receiverMac, (uint8_t*)&dataToSend, sizeof(dataToSend));

    //Serial.print("Sending Letter: ");
    Serial.println(dataToSend.letter);
    //Serial.println(result == ESP_OK ? " ✅ Sent Successfully!" : " ❌ Send Failed!");
}





void setup() {
  WiFi.mode(WIFI_STA);

 

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    } else {
        Serial.println("Peer Added Successfully!");
    }
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    
    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}




void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
       
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print(" ");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(" ");
            Serial.print(ypr[2] * 180/M_PI);
            ypr[0] *= 180/M_PI;
            ypr[1] *= 180/M_PI;
            ypr[2] *= 180/M_PI;
        #endif

      sendData();
      delay(100);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}