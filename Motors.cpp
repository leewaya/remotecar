// 
// 
// 

#include "Motors.h"

#define LEFT_SPEED_DELTA 4
#define MAX_SPEED 220
#define MIN_SPEED 125
#define SPEED_UPDATE_DELAY 50 
#define DIRECTION_UPDATE_DELAY 30 


void MotorsClass::begin(int leftSpeedPin, int leftDirectionPin, int rightSpeedPin, int rightDirectionPin){
	leftSpeedPin_ = leftSpeedPin;
	leftDirectionPin_ = leftDirectionPin;
	rightSpeedPin_ = rightSpeedPin;
	rightDirectionPin_ = rightDirectionPin;

	pinMode(leftSpeedPin, OUTPUT);
	pinMode(leftDirectionPin, OUTPUT);
	pinMode(rightSpeedPin, OUTPUT);
	pinMode(rightDirectionPin, OUTPUT);

	isBegin_ = true;
}

void MotorsClass::updateSpeed(int leftSpeed, int rightSpeed){
	if (!isBegin_) {
		return;
	}

	if (leftSpeed + LEFT_SPEED_DELTA <= MAX_SPEED) {
		leftSpeed_ = leftSpeed + LEFT_SPEED_DELTA;
		rightSpeed_ = rightSpeed;
	}
	else {
		leftSpeed_ = leftSpeed;
		rightSpeed_ = rightSpeed - LEFT_SPEED_DELTA;
	}

	if (leftSpeed_ < MIN_SPEED) {
		leftSpeed_ = 0;
	}

	if (rightSpeed_ < MIN_SPEED) {
		rightSpeed_ = 0;
	}

	if (leftSpeed_ > MAX_SPEED) {
		leftSpeed_ = MAX_SPEED;
	}

	if (rightSpeed_ > MAX_SPEED) {
		rightSpeed_ = MAX_SPEED - LEFT_SPEED_DELTA;
	}


	analogWrite(leftSpeedPin_, leftSpeed_);
	analogWrite(rightSpeedPin_, rightSpeed_);
	delay(SPEED_UPDATE_DELAY);
}
void MotorsClass::updateSpeedPct(int leftSpeedPct, int rightSpeedPct){
	if (!isBegin_) {
		return;
	}

	int16_t leftSpeed = 255 * leftSpeedPct / 100.0;
	int16_t rightSpeed = 255 * rightSpeedPct / 100.0;
	updateSpeed(leftSpeed, rightSpeed);
}

void MotorsClass::moveForward(int speed){
	if (setDirection(DIRECTION_FORWARD)) {
		digitalWrite(leftDirectionPin_, HIGH);
		digitalWrite(rightDirectionPin_, HIGH);
		delay(DIRECTION_UPDATE_DELAY);
	}

	updateSpeed(speed, speed);
}
void MotorsClass::moveBackward(int speed){
	if (setDirection(DIRECTION_BACKWARD)) {
		digitalWrite(leftDirectionPin_, LOW);
		digitalWrite(rightDirectionPin_, LOW);
		delay(DIRECTION_UPDATE_DELAY);
	}

	updateSpeed(speed, speed);
}

void MotorsClass::turnLeft(int speed){
	setDirection(DIRECTION_LEFT);
	updateSpeed(speed, 255);
}

void MotorsClass::turnRight(int speed){
	setDirection(DIRECTION_RIGHT);
	updateSpeed(255, speed);
}

void MotorsClass::turnLeftQuickly(int speed) {
	if (setDirection(DIRECTION_LEFT)) {
		digitalWrite(leftDirectionPin_, LOW);
		digitalWrite(rightDirectionPin_, HIGH);
		delay(DIRECTION_UPDATE_DELAY);
	}

	updateSpeed(speed, speed);
}

void MotorsClass::turnRightQuickly(int speed) {
	if (setDirection(DIRECTION_RIGHT)) {
		digitalWrite(leftDirectionPin_, HIGH);
		digitalWrite(rightDirectionPin_, LOW);
		delay(DIRECTION_UPDATE_DELAY);
	}

	updateSpeed(speed, speed);
}

void MotorsClass::stop(){
	if (setDirection(DIRECTION_STOP)) {
    digitalWrite(leftDirectionPin_, LOW);
    digitalWrite(rightDirectionPin_, LOW);
		updateSpeed(0, 0);
	}
}

bool MotorsClass::changeDirection(uint8_t directionValue, uint16_t speed) {
	if (directionValue < DIRECTIN_FIRST || directionValue > DIRECTION_LAST) {
		return false;
	}

	MOTORS_DIRECTION direction = static_cast<MOTORS_DIRECTION>(directionValue);	
	switch (direction) {
	case DIRECTION_STOP:
		stop();
		break;
	case DIRECTION_FORWARD:
		moveForward(speed);
		break;
	case DIRECTION_BACKWARD:
		moveBackward(speed);
		break;
	case DIRECTION_LEFT:
		turnLeft(speed);
		break;
	case DIRECTION_RIGHT:
		turnRight(speed);
		break;
	default:
		break;
	}

	return true;
}

bool MotorsClass::setDirection(MOTORS_DIRECTION direction) {
	if (direction_ == direction) {
		return false;
	}
	direction_ = direction;
	return true;
}


MotorsClass Motors;


