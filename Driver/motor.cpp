#include "motor.h"

const uint16_t  MOTOR_PWM_PIN[MOTORn] = {MOTOR1_PWM_Pin, MOTOR2_PWM_Pin, MOTOR3_PWM_Pin, MOTOR4_PWM_Pin,};
const uint16_t  MOTOR_A_PIN[MOTORn] = {MOTOR1_A_PIN, MOTOR2_A_PIN, MOTOR3_A_PIN, MOTOR4_A_PIN,};
const uint16_t  MOTOR_B_PIN[MOTORn] = {MOTOR1_B_PIN, MOTOR2_B_PIN, MOTOR3_B_PIN, MOTOR4_B_PIN};

Motor::Motor(Motor_TypeDef _motor, uint16_t _arr, uint32_t _psc)
{
	motor = _motor;
	arr = _arr;  //254
	psc = _psc;  //575
}

void Motor::init()
{
    PWM_Init(MOTOR_PWM_PIN[this->motor], arr, psc);
    pinMode(MOTOR_A_PIN[this->motor], OUTPUT);
    pinMode(MOTOR_B_PIN[this->motor], OUTPUT);

}

void Motor::spin(int pwm)
{
    if(pwm > 0){
	  digitalWrite(MOTOR_A_PIN[this->motor], LOW);
		//digitalWrite(MOTOR_B_PIN[this->motor], LOW);
	}else if(pwm < 0) {
		digitalWrite(MOTOR_A_PIN[this->motor], HIGH);
		//digitalWrite(MOTOR_B_PIN[this->motor], HIGH);
	}
	if(this->motor == MOTOR1){
		 analogWrite(MOTOR_PWM_PIN[this->motor], abs(pwm));
	}
	if(this->motor == MOTOR2){
	   analogWrite(MOTOR_PWM_PIN[this->motor], abs(pwm));
	}
	if(this->motor == MOTOR3){
		 analogWrite(MOTOR_PWM_PIN[this->motor], abs(pwm));
	}
	if(this->motor == MOTOR4){
		 analogWrite(MOTOR_PWM_PIN[this->motor], abs(pwm));
	}
}


void Motor::updateSpeed(long encoder_ticks)
{
	//this function calculates the motor's RPM based on encoder ticks and delta time
	unsigned long current_time = millis();
	unsigned long dt = current_time - prev_update_time_;

	//convert the time from milliseconds to minutes
	double dtm = (double)dt / 60000;
	double delta_ticks = encoder_ticks - prev_encoder_ticks_;

	//calculate wheel's speed (RPM)
	rpm = (delta_ticks / 1560) / dtm;

	prev_update_time_ = current_time;
	prev_encoder_ticks_ = encoder_ticks;
}

