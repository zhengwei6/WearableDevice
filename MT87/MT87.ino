#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SPI.h>                                    // For SD 
#include <SD.h>                                    // For SD ���蕭嚙�

#include <Timer.h>

#include <EEPROM.h>                              // ���蕭謖���蕭謅��雓Ｗ�蕭�嚙踐垓嚙踝�ㄡ���蕭ile


//#define SERIALOUPUT
#define SDOUTPUT


File myFile;  // SD card
Sd2Card card;  // SD card
Timer t;

/*-- ��������蕭 -----------------------------------------------------------------------------------------------*/
volatile boolean DEV_state = LOW;  // ���蕭謖���蕭謕竣���������
int ID;
/*-- EEPROM ���蕭蹎刻���-----------------------------------------------------------------------------------------*/

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
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
VectorInt16 gyro;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Serial.begin(9600);  // ���嚙踝蕭豲鞈對�雓�嚙踐嚙踐���雓�頩�嚙踝蕭豲��蕭
    // 嚙踝蕭�������蕭 EEPROM


    int value = EEPROM.read(5);
    if (value == 255){
      value = 0;
    }
    ID = value + 1;
    EEPROM.write(5, ID);

    String filename = "TEST"+String(ID)+".csv";
    Serial.print("Test file:");
    Serial.println(filename);
    
    // ���嚙踝蕭豲��蕭 SD ���蕭嚙�
    if (!SD.begin(4)) {
       Serial.println("SD Fail!");
       return;
     }
    myFile = SD.open(filename, FILE_WRITE);
    Serial.println(myFile);  
    if (myFile) {                                   // ���嚙踝�蕭嚙踐��蕭嚙踝�嚙踝蕭��嚙踝蕭豲鞈察�腦嚙踐��蕭
       Serial.println("Write to "+filename+"...");         
       myFile.close();                               // ���雓捕�嚙踐���蕭����蕭
    } else {
       Serial.println("\n open file error ");    // ���蕭��蕭謚啾馭���蕭豲鞈察縈�頩偌嚙踝����蕭��狗��蕭
    }
    // 嚙踐�蕭嚙踐�雓嚙踝�雓蕭
    pinMode(3, INPUT_PULLUP);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-3842);
    mpu.setYAccelOffset(59);
    mpu.setZAccelOffset(1295);
    mpu.setXGyroOffset(-29);
    mpu.setYGyroOffset(25);
    mpu.setZGyroOffset(52);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
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

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    while(!digitalRead(3)){
       delay(20);
       if(digitalRead(3)){
            DEV_state = !DEV_state;
            if(DEV_state == 1){
                 String filename = "TEST"+String(ID)+".csv";
                 myFile = SD.open(filename, FILE_WRITE);
                 if (myFile) {                                   // ���嚙踝�蕭嚙踐��蕭嚙踝�嚙踝蕭��嚙踝蕭豲鞈察�腦嚙踐��蕭
                     Serial.println("Write to "+filename+"...");         
                 } else {
                     Serial.println("\n open file error ");    // ���蕭��蕭謚啾馭���蕭豲鞈察縈�頩偌嚙踝����蕭��狗��蕭
                }
            }
           else{
                 Serial.println("File close");  
                 myFile.close();
           }
       }
    }
    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if(DEV_state){
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        unsigned long time =  millis(); 
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        float tg = 250.0/32767.0;
        float ta = 2.0/32767.0;
        // display real acceleration, adjusted to remove gravity
        #ifdef SERIALOUPUT
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          Serial.print(time);Serial.print(" ");
          Serial.print(float(aaReal.x)*ta); Serial.print(" ");
          Serial.print(float(aaReal.y)*ta); Serial.print(" ");
          Serial.print(float(aaReal.z)*ta); Serial.print("  ");
          
          mpu.dmpGetGyro(&gyro,fifoBuffer);
          Serial.print(float(gyro.x)*tg); Serial.print(" ");
          Serial.print(float(gyro.y)*tg); Serial.print(" ");
          Serial.print(float(gyro.z)*tg); Serial.print("  ");
          
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          Serial.print(ypr[0] * 180/M_PI);
          Serial.print(" ");
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print(" ");
          Serial.println(ypr[2] * 180/M_PI);
        #endif
        #ifdef SDOUTPUT
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          myFile.print(time);myFile.print(",");
          myFile.print(float(aaReal.x)*ta); myFile.print(",");
          myFile.print(float(aaReal.y)*ta); myFile.print(",");
          myFile.print(float(aaReal.z)*ta); myFile.print(",");
  
          mpu.dmpGetGyro(&gyro,fifoBuffer);
          myFile.print(float(gyro.x)*tg); myFile.print(",");
          myFile.print(float(gyro.y)*tg); myFile.print(",");
          myFile.print(float(gyro.z)*tg); myFile.print(",");
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          myFile.print(ypr[0] * 180/M_PI);
          myFile.print(",");
          myFile.print(ypr[1] * 180/M_PI);
          myFile.print(",");
          myFile.println(ypr[2] * 180/M_PI);
         #endif
    }
    } 
}





