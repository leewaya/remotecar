#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>  //声明伺服马达函数库
#include "Motors.h"

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        13 //14    
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        12  //17

#define HORIZONTAL_SERVO    8
#define VERTICAL_SERVO      9

#define LEFT_SPEED_PIN 5   
#define RIGHT_SPEED_PIN 6   
#define LEFT_DIRECTION_PIN 4     
#define RIGHT_DIRECTION_PIN 7  


#define MIN_SPEED 125
#define MAX_SPEED 255
#define SPEED_DELTA 50

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

int range = 30;               // output range of X or Y movement
int threshold = range/4;      // resting threshold
int center = range/2;         // resting position value

int speedRange = 255*2;               // output range of X or Y movement
int speedThreshold = speedRange/4;      // resting threshold
int speedCenter = speedRange/2;         // resting position value


Servo hortizontalServo;  // 定义伺服马达对象变量
Servo verticalServo;  // 定义伺服马达对象变量

int hortizontalServoAngle;
int verticalServoAngle;

int mSpeed=0;
bool isTurning=false;

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

// Reset func 
void (* resetFunc) (void) = 0;

void setup(){
 
  Serial.begin(115200);
  
  delay(500);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");
  Serial.print("rumble = ");
  if (rumble)
    Serial.println("true)");
  else
    Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.println("Unknown Controller type found ");
      break;
    case 1:
      Serial.println("DualShock Controller found ");
      break;
    case 2:
      Serial.println("GuitarHero Controller found ");
      break;
  case 3:
      Serial.println("Wireless Sony DualShock Controller found ");
      break;
   }

   hortizontalServo.attach(HORIZONTAL_SERVO);  
   verticalServo.attach(VERTICAL_SERVO); 

   hortizontalServoAngle = 90;
   verticalServoAngle = 10;

   hortizontalServo.write(hortizontalServoAngle);//舵机停转
   verticalServo.write(verticalServoAngle);//舵机停转

   Motors.begin(LEFT_SPEED_PIN, LEFT_DIRECTION_PIN, RIGHT_SPEED_PIN, RIGHT_DIRECTION_PIN);
}

void stop() {
  Motors.stop();
  mSpeed = 0;
  isTurning = false;
}

void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */  
  if(error == 1){ //skip loop if no controller found
   resetFunc();
   stop();
   return;
  }

   ps2x.read_gamepad(false, 0);          //read controller and set large motor to spin at 'vibrate' speed

  if(ps2x.NewButtonState(PSB_BLUE)) {
       stop();
       return;   
  }
  
   // read and scale the two axes:
  int rightX = readAxis(PSS_RX, false);
  int rightY = readAxis(PSS_RY, false);

//   Serial.print("rightY:");
//   Serial.println(rightY);
  if (abs(rightY) == 4) {
    rightY = 0;//Fix left joystick issue. rightY is not 255/2 when in idle state
  }
 
   int oldHortizontal = hortizontalServoAngle;
   hortizontalServoAngle = getAccumulatedValue(hortizontalServoAngle, -rightX, 75, 15);  
   if (hortizontalServoAngle != oldHortizontal) {
        hortizontalServo.write(hortizontalServoAngle);
        delay(50);
    }
   // Serial.print("hortizontalServoAngle:");
   // Serial.println(hortizontalServoAngle);
         
  int oldVertical = verticalServoAngle;
   verticalServoAngle = getAccumulatedValue(verticalServoAngle, -rightY, 110, 10);
   if (oldVertical != verticalServoAngle) {
      verticalServo.write(verticalServoAngle);
       delay(50);
   }

 //   Serial.print("verticalServoAngle:");
 //   Serial.println(verticalServoAngle);
      
  int leftX =  ps2x.Analog(PSS_LX);
  int leftY = ps2x.Analog(PSS_LY);

   Serial.print("leftY:");
   Serial.println(leftY);

   Serial.print("leftX:");
   Serial.println(leftX);

   if (leftX > 150) {
        isTurning = true;
        Motors.turnLeftQuickly(150);
   } 
   else if (leftX < 100) {
       isTurning = true;       
       Motors.turnRightQuickly(150);
   }
   else {
      stop();
   }

  if (!isTurning) {
   if (leftY<120) {
        int newSpeed = MAX_SPEED - leftY;
        if (abs(newSpeed - mSpeed) > SPEED_DELTA) {
           mSpeed = newSpeed;        
           Motors.moveForward(mSpeed);
        }
    }
   else if (leftY > 130) {
        if (abs(leftY - mSpeed) > SPEED_DELTA) {
           mSpeed = leftY;
           Motors.moveBackward(leftY);
        }
    }
    else {
      stop();
    }
  }
   
   delay(50);
}

int getAccumulatedValue(int currentValue, int delta, int maxValue, int minValue) {
  currentValue = currentValue + delta;
   if (currentValue <= minValue) {
      currentValue = minValue;
   }
   else if (currentValue >= maxValue) {
       currentValue = maxValue;
   }
   
   return currentValue;
}
/*
  reads an axis (0 or 1 for x or y) and scales the 
 analog input range to a range from 0 to <range>
 */

int readAxis(int thisAxis, bool showReading) { 
  // read the analog input:
  int reading = ps2x.Analog(thisAxis);

  if (showReading) {
    Serial.print("reading:");
    Serial.println(reading);
  }
  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 255, 0, range);

  // if the output reading is outside from the
  // rest position threshold,  use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  } 

  // return the distance for this axis:
  return distance;
}

