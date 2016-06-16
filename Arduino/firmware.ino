#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MotorShield.h>


/*
 * version 2.2
 * change log: support run backward
 */
 
/*
 * version 2.1.1
 * change log: allow the arm to touch the ground, decrease arm&clamp tolerence
 */
 
/*
 * version 2.1
 * change log: ignore illegal command chars, do action only llegal chars received
 */

// PINS USED: 2,3,4,5,8,9,10

// ******* BLE BOARD ******//
#define RxD 2
#define TxD 3
//#define BLE Serial

// ******* SONAR SENSOR ******//
#define trigPin 6
#define echoPin 7
 
#define DEBUG_ENABLED  0

#define SVO_PIN_EYE  8
#define SVO_PIN_ARM  9
#define SVO_PIN_CLP  10

#define RSP_FLAG  0xF1

//FMT:    MMAABBCCDD
//MODE:   MM(0xFF:lock, 0xF0:manual, 0xF1:auto)
//STEER:  AA(0xFF:forward, 0xA1:left, 0xA2:right, 0x00:backward)
//GAS:    BB(0xFF:park, 0xB1-5:shift 1-5)
//CLAMP:  CC(0x00-0xFF)
//ARM:    DD(0x00-0xFF)

#define DATA_PKG_LEN  5

#define DRIVE_LOCK      0xFF
#define DRIVE_MANUAL    0xF0
#define DRIVE_AUTO      0xF1

// ******* STEER CONTROL ******//
#define GO_LEFT     0xA1
#define GO_RIGHT    0xA2
#define GO_FORWARD  0xFF
#define GO_BACKWARD 0x00

// ******* SHIFT ******//
#define SFT_P    0xFF
#define SFT_1    0xB1
#define SFT_2    0xB2
#define SFT_3    0xB3
#define SFT_4    0xB4
#define SFT_5    0xB5

// ********** SPEED OFFSET ********** // Motors with encoder are recommended to avoid doing this manually
#define SPEED_OFFSET_SFT1         0
#define SPEED_OFFSET_SFT2         6
#define SPEED_OFFSET_SFT3         0
#define SPEED_OFFSET_SFT4         2
#define SPEED_OFFSET_SFT5         0

#define DEFAULT_MOTOR_SPEED  254


// ********** INITIALIZE **********// 


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield with the default I2C addr
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);     //left motor on M3
Adafruit_DCMotor *motorRight = AFMS.getMotor(4);    //right motor on M4

SoftwareSerial BLE(RxD,TxD);        //BLE pins

//byte STEER;                       //Current direction
byte SHIFT;                         //Current shift
byte CLAMP;                         //Current clamp angle
byte ARM;                           //Current arm angle

byte MODE_CMD;                      //Drive mode command
byte STEER_CMD;                     //Direction command
byte SHIFT_CMD;                     //Shift command
byte CLAMP_CMD;                     //Clamp command
byte ARM_CMD;                       //Arm command


long distance = 0;
long leftSpeed, rightSpeed;         //Motor speeds

Servo servoEye;
Servo servoArm;
Servo servoClp;

int eyePos[4] = {60,90,120,90};
int currentPos = 0;
unsigned long previousMillis;

void setup() 
{ 
  
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  AFMS.begin();                       // Motor Shield. create with the default frequency 1.6KHz
  Serial.begin(9600);
  BLE.begin(9600);                    //Set BLE BaudRate to default baud rate 9600
  
  //pinMode(trigPin,OUTPUT);            //set ultrasonic sensor pins
  //pinMode(echoPin,INPUT);
  
  motorLeft->setSpeed(DEFAULT_MOTOR_SPEED);            //set default speed
  motorRight->setSpeed(DEFAULT_MOTOR_SPEED);
  motorLeft->run(RELEASE);                             //start motors
  motorRight->run(RELEASE);
} 
 
void loop() 
{ 
    byte buff[DATA_PKG_LEN];
    // check if there's any data sent from the remote BLE shield
    if(BLE.available()>=0){
    // illegal chars will be read to the buffer when Phone BLE is offline           
       BLE.readBytes(buff,DATA_PKG_LEN);
       int i=0;
       // READ COMMANDs  
       MODE_CMD  = buff[i++];
       STEER_CMD = buff[i++];
       SHIFT_CMD = buff[i++];
       CLAMP_CMD = buff[i++];
       ARM_CMD   = buff[i++];
       BLE.flush();
    }

// ********* DEBUG INFO ************//

#if DEBUG_ENABLED 
      Serial.println("0x");
      Serial.println(MODE_CMD,HEX);
      Serial.println(STEER_CMD,HEX);
      Serial.println(SHIFT_CMD,HEX);
      Serial.println(CLAMP_CMD,HEX);
      Serial.println(ARM_CMD,HEX);
      Serial.println(":");
      if(STEER_CMD==GO_RIGHT){
         Serial.println("-->");
       }else if(STEER_CMD==GO_LEFT){
         Serial.println("<--");
       }else{
         Serial.println("^");
       }
#endif

  /* illegal chars will be read to the buffer when Phone BLE is offline
   * do actions only llegal mode command received
   */ 
  //if( MODE_CMD==DRIVE_LOCK || MODE_CMD==DRIVE_MANUAL || MODE_CMD==DRIVE_AUTO){
      armAction();
      clampAction();
      detectDistance();
      //scanAround();
      
      switch (MODE_CMD){
      case DRIVE_LOCK:
        stop();
        break;
      case DRIVE_MANUAL:
        shift();        // shift and move
        if(distance>0 and distance <20){
            stayBack();
         } 
        break;
      case DRIVE_AUTO:
            autoDrive();
        break;
      }
      
      BLE.write(RSP_FLAG);
      BLE.write(byte(distance));
      BLE.write(byte(leftSpeed));
      BLE.write(byte(rightSpeed));
//    }else{
//#if DEBUG_ENABLED 
//        Serial.println("illegal mode command received");
//#endif
//    }
} 
 
 
//************************ PRIVATE METHODS *****************// 

void shift(){
      // *************** GAS CONTROL ************//
      if(SHIFT != SHIFT_CMD){    // check if need to shift
        SHIFT = SHIFT_CMD;       // sync current shift
        switch (SHIFT){
          case SFT_P:
            leftSpeed = 0;
            rightSpeed = 0;
            break;
          case SFT_1:
            leftSpeed = 135 + SPEED_OFFSET_SFT1;
            rightSpeed = 135;
            break;
          case SFT_2:
            leftSpeed = 155 + SPEED_OFFSET_SFT2;
            rightSpeed = 155;
            break;
          case SFT_3:
            leftSpeed = 200 + SPEED_OFFSET_SFT3;
            rightSpeed = 200;
            break;
          break;
          case SFT_4:
            leftSpeed = 225 + SPEED_OFFSET_SFT4;
            rightSpeed = 225;
            break;
          break;
          case SFT_5:
            leftSpeed = 255;
            rightSpeed = 249;
            break;
        }
        motorLeft->setSpeed(leftSpeed);
        motorRight->setSpeed(rightSpeed);
        delay(10);
        
        // ******** STEERING ***********
        if(SHIFT!=SFT_P){
          steer();                // steering
        }else{
          motorLeft->run(RELEASE);  // stop
          motorRight->run(RELEASE); // stop
          delay(10);
        }
    }
}

void steer(){
      // *********** DIRECTION CONTROL *************//
      //if(STEER != STEER_CMD){   // check if need to change direction
        //STEER = STEER_CMD;      // sync current direction
        switch (STEER_CMD){
          case GO_FORWARD:
            motorLeft->run(FORWARD);
            motorRight->run(FORWARD);
            break;
          case GO_BACKWARD:
            motorLeft->run(BACKWARD);
            motorRight->run(BACKWARD);
            break;
          case GO_LEFT:
            motorLeft->run(BACKWARD);
            //motorLeft->run(RELEASE);
            motorRight->run(FORWARD);
            break;
          case GO_RIGHT:
            motorLeft->run(FORWARD);
            motorRight->run(BACKWARD);
            //motorRight->run(RELEASE);
            break;
        }
        delay(10);
    //}
}

void clampAction(){
  // accurate offset
  if(abs(CLAMP-CLAMP_CMD) > 3){
      CLAMP=CLAMP_CMD;
      int pos = map(CLAMP,0,255,66,0);
      servoClp.attach(SVO_PIN_CLP);
      delay(5);
      servoClp.write(pos);
      delay(150);
      servoClp.detach();
  }
}

void armAction(){
  // accurate offset
  if(abs(ARM-ARM_CMD) > 3){
      ARM=ARM_CMD;
      //int pos = map(ARM,0,255,0,142);
      int pos = map(ARM,0,255,0,160);
//      Serial.print("arm val:");
//      Serial.println(ARM);
//      Serial.print("arm pos:");
//      Serial.println(pos);
      
      servoArm.attach(SVO_PIN_ARM);
      delay(5);
      servoArm.write(pos);
      delay(150);
      servoArm.detach();
  }
}

void scanAround(){
//  unsigned long currentMillis = millis();
//  if((currentMillis - previousMillis) >= 200){
//    previousMillis = currentMillis;
//    if(currentPos >= 3){  
//        currentPos=0;
//      }
//    Serial.print("syePos:");
//    Serial.println(eyePos[currentPos]);
//    servoEye.attach(SVO_PIN_EYE);
//    delay(10);
//    servoEye.write(eyePos[currentPos++]);  
//    delay(100);
//  }

    if(currentPos >= 3){  
        currentPos=0;
      }
    Serial.print("syePos:");
    Serial.println(eyePos[currentPos]);
    //servoEye.attach(SVO_PIN_EYE);
    //delay(10);
    servoEye.write(eyePos[currentPos++]);  
    delay(200);
  
}


void detectDistance(){
    long duration;
 // DISTANCE DETECT CONFIG START
    digitalWrite(trigPin,LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin,LOW);
    duration = pulseIn(echoPin,HIGH,100000);  //100 ms  Timeout is very important

#if DEBUG_ENABLED 
    Serial.print("duration:");
    Serial.println(duration);
#endif
    
    // DISTANCE DETECT CONFIG END
    distance = (duration/2) / 29.1;
    delay(5);
}

void stayBack(){
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  motorLeft->setSpeed(155);
  motorRight->setSpeed(155);
  motorLeft->run(BACKWARD);
  motorRight->run(BACKWARD);
  delay(500);
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

void autoDrive(){
  motorLeft->setSpeed(175);
  motorRight->setSpeed(175);
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
  if(distance>0 and distance <15){
      stayBack();
      long randnum = random(1,10);
      if(randnum % 2){
        motorLeft->run(BACKWARD);
        motorRight->run(FORWARD);
        delay(1000);
      }else{
        motorLeft->run(FORWARD);
        motorRight->run(BACKWARD);
        delay(700);
      }
  }
}

void stop(){
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

//********************** BLE CONFIG ***********************
// This is used to initialize & congifure the BLE shield for the FIRST time ONLY
// Modify it to fit your needs, you can also configure your BLE shild out of Arduino

void setupBleConnection()
{
  BLE.begin(9600); //Set BLE BaudRate to default baud rate 9600
  delay(3000);
  BLE.print("AT+CLEAR"); //clear all previous setting
  delay(1000);
  readBluetooth();
  BLE.print("AT+NAMEOscar.Robot"); //clear all previous setting
  readBluetooth();
  
  BLE.print("AT+ROLE0"); //set the bluetooth name as a slaver
  readBluetooth();
  BLE.print("AT+ROLE?"); //set the bluetooth name as a slaver
  readBluetooth();
  BLE.print("AT+SAVE1");  //don't save the connect information
  readBluetooth();
  BLE.print("AT+SAVE?");  //don't save the connect information
  readBluetooth();
  BLE.print("AT+BAUD?");  //don't save the connect information
  readBluetooth();
  BLE.print("AT+CHAR?");  
  readBluetooth();
}

void readBluetooth(){
  delay(1000);  
  char recvChar=0;
  String recvBuf="";
    while(BLE.available()){
      recvChar = BLE.read();
 	recvBuf += recvChar;
    }
  //Serial.println(recvBuf);
  //BLE.flush();
}
