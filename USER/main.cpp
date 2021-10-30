#include <Arduino.h>


#include <Gy85.h>
#include <elapsedMillis.h>
#include <PID.h>
#include <MOTOR.h>
#include <encoder.h>
#include <battery.h>

#include <Kinematics.h>
#include <ros.h>
#include <riki_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Imu.h>
#include <riki_msgs/Battery.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include <ODriveArduino.h>
#include "CommonMacro.h"

HardwareSerial& odrive_serial = Serial2;
HardwareSerial& odrive_serial1 = Serial6;

ODriveArduino odrive(odrive_serial);
ODriveArduino odrive1(odrive_serial1);

bool accel, gyro, mag;
bool is_first = true;

double required_angular_vel_z = 0;
double required_linear_vel_x = 0;
double required_linear_vel_y = 0;
uint32_t previous_command_time = 0;


//bool OnOff = true;


uint32_t previous_battery_debug_time = 0;
uint32_t previous_debug_time = 0;
uint32_t previous_imu_time = 0;
uint32_t previous_control_time = 0;
uint32_t publish_vel_time = 0;
char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";


double gx, gy, gz;
double ax, ay, az;

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

PID motor1_pid(-255, 255, K_P, K_I, K_D);
PID motor2_pid(-255, 255, K_P, K_I, K_D);
PID motor3_pid(-255, 255, K_P, K_I, K_D);
PID motor4_pid(-255, 255, K_P, K_I, K_D);


Gy85  imu2;
Battery bat(25, 10.6, 12.6);

Motor motor1(MOTOR1, 255, 490);
Motor motor2(MOTOR2, 255, 490);
Motor motor3(MOTOR3, 255, 490);
Motor motor4(MOTOR4, 255, 490);
void pid_callback( const riki_msgs::PID& pid);
void command_callback( const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle  nh;

riki_msgs::Imu raw_imu_msg;
riki_msgs::Velocities raw_vel_msg;
riki_msgs::Battery raw_battery_msg;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);


void pid_callback( const riki_msgs::PID& pid)
{
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
	required_linear_vel_x = cmd_msg.linear.x;
	required_linear_vel_y = cmd_msg.linear.y;
	required_angular_vel_z = cmd_msg.angular.z;

    previous_command_time = millis();
}
void move_base()
{

    
    Kinematics::rpm req_rpm = kinematics.getRPM(required_linear_vel_x, required_linear_vel_y, required_angular_vel_z);
 
//	  motor1.spin(req_rpm.motor1);
//    motor2.spin(req_rpm.motor2);
//    motor3.spin(req_rpm.motor3);
//    motor4.spin(req_rpm.motor4);
  
    odrive1.SetVelocity(1, req_rpm.motor1);
    odrive1.SetVelocity(0, -req_rpm.motor2);
    odrive.SetVelocity(1, req_rpm.motor3);
    odrive.SetVelocity(0, -req_rpm.motor4);
	  Kinematics::velocities current_vel;
 
    current_vel = kinematics.getVelocities(odrive1.GetVelocity(1), -(odrive1.GetVelocity(0)), odrive.GetVelocity(1), -(odrive.GetVelocity(0)));
	
	 //fill in the object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg object to ROS
    raw_vel_pub.publish(&raw_vel_msg);
}   
void check_imu()
{
    gyro = imu2.check_gyroscope();
    accel = imu2.check_accelerometer();
    mag = imu2.check_magnetometer();

    if (!accel){
        nh.logerror("Accelerometer NOT FOUND!");
    }   

    if (!gyro){
        nh.logerror("Gyroscope NOT FOUND!");
    }   

    if (!mag){
        nh.logerror("Magnetometer NOT FOUND!");
    }
    is_first = false;
}
//geometry_msgs::Vector3 acceler, gyro, mag;
 //this function publishes raw IMU reading
void publish_imu()
{
	// imu.readSensor();   
	//measure accelerometer  
  imu2.measure_acceleration(); 
	//measure gyroscope
	imu2.measure_gyroscope();
	imu2.measure_magnetometer();
	//publish raw_imu_msg object to ROS
  raw_imu_msg.linear_acceleration = imu2.raw_acceleration;  
  raw_imu_msg.magnetic_field = imu2.raw_magnetic_field;
  raw_imu_msg.angular_velocity = imu2.raw_rotation;
  raw_imu_pub.publish(&raw_imu_msg);
}

void stop_base()
{
	required_linear_vel_x = 0;
	required_linear_vel_y = 0;
	required_angular_vel_z = 0;
}

void publishBAT()
{
	raw_battery_msg.battery = bat.get_volt();
	raw_battery_pub.publish(&raw_battery_msg);
}

void print_debug()
{
    char buffer[50]; 

	  sprintf (buffer, "motor1  : %f", odrive1.GetVelocity(1));
    nh.loginfo(buffer);	
    sprintf (buffer, "motor2 : %f", odrive1.GetVelocity(0));
    nh.loginfo(buffer);
	  sprintf (buffer, "motor3  : %f", odrive.GetVelocity(1));
    nh.loginfo(buffer);	
    sprintf (buffer, "motor4 : %f", odrive.GetVelocity(0));
    nh.loginfo(buffer);
   
}
//void Serial_EventHandler()
//{
//    togglePin(LED_Pin);
//}

void setup() {


//	 motor1.init();
//	 motor2.init();
//	 motor3.init();
//	 motor4.init();
//   encoder_init();
	 pinMode(LED_Pin, OUTPUT);
	 //bat.init();
	 //imu.begin();
	 imu2.init();

	 // ODrive uses 115200 baud
  Serial2.begin(115200);
	Serial6.begin(115200);
 //Serial6.attachInterrupt(Serial_EventHandler);
	  nh.initNode();	
	  nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
  //  nh.advertise(raw_battery_pub);
   // nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);

	//while (!nh.connected()){
		

    __IntervalExecute(togglePin(LED_Pin), 2000);
		nh.spinOnce();
//	}   
	nh.loginfo("Robot Connected YES !!!");
	digitalWrite(LED_Pin, LOW);


}

void loop() {
    
	if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
			 move_base();
            previous_control_time = millis();
        }

        if ((millis() - previous_command_time) >= 400){
            stop_base();
        }
				
//      if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
//              
//                        //publish the IMU data
//                  if(is_first){
//                        //sanity check if the IMU exits
//                        check_imu();
//                } else{
//                        //publish the IMU data
//                        publish_imu();
//                }
//                previous_imu_time = millis();
//        }
    
//        if( (millis() - previous_battery_debug_time) >= (1000 / BAT_PUBLISH_RATE)){
//            if(bat.get_volt() < 11.300000f){
//                
//                nh.logwarn(battery_buffer);			
//            }
//            publishBAT();
//            previous_battery_debug_time = millis();		
//        }

//        if(DEBUG){
//            if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {
//                print_debug();
//                previous_debug_time = millis();
//            }
//        }
        nh.spinOnce();

}

/**
  * @brief  Main Function
  * @param  None
  * @retval None
  */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    ADCx_Init(ADC1);
    setup();
    for(;;)loop();
}
