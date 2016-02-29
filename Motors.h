// Motors.h

#ifndef _MOTORS_h
#define _MOTORS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

enum MOTORS_DIRECTION {
	DIRECTIN_FIRST = 0, 
	DIRECTION_STOP = 0, 
	DIRECTION_FORWARD = 1, 
	DIRECTION_BACKWARD = 2, 
	DIRECTION_LEFT = 3, 
	DIRECTION_RIGHT = 4, 
	DIRECTION_LAST = 4 
};


class MotorsClass
{
 
 public:
	 MotorsClass() :
		 isBegin_(false), leftSpeedPin_(0), leftDirectionPin_(0), rightSpeedPin_(0), rightDirectionPin_(0),
		 leftSpeed_(0), rightSpeed_(0), direction_(DIRECTION_STOP)
	 {};

	 void begin(int leftSpeedPin, int leftDirectionPin, int rightSpeedPin, int rightDirectionPin);
	 void moveForward(int speed);
	 void moveBackward(int speed);
	 void turnLeft(int speed);
	 void turnRight(int speed);
	 void turnLeftQuickly(int speed);
	 void turnRightQuickly(int speed);
	 void stop();

	 void updateSpeedPct(int speedLeftPct, int speedRightPct);
	 void updateSpeed(int speedLeft, int speedRight);

	 bool changeDirection(uint8_t directionValue, uint16_t speed);
	 inline MOTORS_DIRECTION getCurrentDirection() { return direction_; };
	 inline bool isForward() { return direction_ == DIRECTION_FORWARD; };
	 inline bool isBackward() { return direction_ == DIRECTION_BACKWARD; };
	 inline bool isStop() { return direction_ == DIRECTION_STOP; };

	 uint16_t getLeftSpeed() { return leftSpeed_; };
	 uint16_t getRightSpeed() { return rightSpeed_; };
	
private:
	bool setDirection(MOTORS_DIRECTION direction);

	bool isBegin_;
	int leftSpeedPin_;
	int leftDirectionPin_;
	int rightSpeedPin_;
	int rightDirectionPin_;	
	uint16_t leftSpeed_;
	uint16_t rightSpeed_;
	MOTORS_DIRECTION direction_;
};

extern MotorsClass Motors;

#endif


