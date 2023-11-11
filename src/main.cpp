#include "main.h"
#include "display/lv_objx/lv_list.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "pros/vision.h"
#include <string>
#include <cmath>
#include "MarinRoboticsCustom/math_utils.h"
//motor initialization
using pros::E_CONTROLLER_ANALOG_LEFT_Y;
using pros::E_CONTROLLER_ANALOG_RIGHT_Y;
using namespace std;
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Custom Variables
  // Drive Controls
bool drive_plus_turning = true; 
bool drive_plus_forward = true; 
bool standard_drive = true;
pros::ADIDigitalOut piston ('A');
bool pistonval = false;
  // Auton
float Wheel_Diameter = 4;
float Turning_Diameter = 13;
float Turn_Tuning_Factor = 2.1;
float Move_Tuning_Factor = 2.2;
  // Slew Rate Limiting
float up_step = 50;
float down_step = -50;
float turn_constant = 0.7;

// Static Variables
  // Main Drive
  int left_x, left_y, right_x, right_y;
  // Auton Drive Vars
float Wheel_Circumference = Wheel_Diameter * 3.1416;
float Turning_Circumference = Turning_Diameter * 3.1416;
float Turning_Distance, Wheel_Revolutions, Turn_Wheel_Rotation, Forward_Wheel_Rotation;
bool wait = false;
  // Slew Rate Limiting
float y_current, x_current, y_direction, x_direction;
float y_true_step;
float x_true_step;

pros::Motor front_left_motor(17, pros::E_MOTOR_GEAR_BLUE, true,
                             pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left_motor(16, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right_motor(19, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right_motor(20, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor catapult_motor_1(1, pros::E_MOTOR_GEAR_RED, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor catapult_motor_2(8, pros::E_MOTOR_GEAR_RED, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor snarf_rotator_1( 2,pros::E_MOTOR_GEAR_GREEN, false, pros:: E_MOTOR_ENCODER_DEGREES );
pros::Motor snarf_rotator_2( 6,pros::E_MOTOR_GEAR_GREEN, true, pros:: E_MOTOR_ENCODER_DEGREES );
pros::Motor snarf_driver(4, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
//grouping both sides of the drivetrain
pros::MotorGroup left_motors({front_left_motor, back_left_motor});
pros::MotorGroup right_motors({front_right_motor, back_right_motor});
pros::MotorGroup snarf_rotator({snarf_rotator_1, snarf_rotator_2});
pros::MotorGroup catapult_motor({catapult_motor_1, catapult_motor_2});
int repeat=0;
//modifier from 127 to 600
float modifier = 600.0/127;
//just the square root of 2
float rt2 = sqrt(2.0);
//defining the controller
int auton_step = 0; // debug

void debug_auton(int step){
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "Running Auton");
  pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Step: %d", step);
}

float extenddistance = 180;
bool isextended = false;
bool snarferon = false;
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	} 
}
float move_factor= 360/Wheel_Circumference;


float autonselect=1 ;
pros::Mutex action;
void move( float inches, float velocity) { 	//action.take(1000);
	Forward_Wheel_Rotation = Move_Tuning_Factor*(inches/Wheel_Circumference)*360;

  left_motors.tare_position();
  right_motors.tare_position();

  left_motors.move_relative(Forward_Wheel_Rotation,velocity);
  right_motors.move_relative(Forward_Wheel_Rotation,velocity);
		//action.give();
	}
void turn (float angle, float velocity) {
	//action.take(1000);
	 Turning_Distance = angle/360 * Turning_Circumference;
  Wheel_Revolutions = Turning_Distance/Wheel_Circumference;
  Turn_Wheel_Rotation = Turn_Tuning_Factor*Wheel_Revolutions*360; // Degrees that the wheel should turn
  left_motors.move_relative(Turn_Wheel_Rotation*autonselect, velocity);
  right_motors.move_relative(-Turn_Wheel_Rotation*autonselect, velocity);

}

//change to negative if on right
void initialize() {
	
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	catapult_motor.set_brake_modes(MOTOR_BRAKE_HOLD);
	pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  catapult_motor.set_brake_modes(MOTOR_BRAKE_HOLD);
  	snarf_rotator_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	snarf_rotator_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
move(-28, 500);
pros::delay(1500);
turn(45, 600 );
pros::delay(400);
  left_motors.move_velocity(-600);
  right_motors.move_velocity(-600);
  pros::delay(500);

//pros::Task body1{ 
//  [=]{
  //  action.take(5000);
  move(6, 600);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                              "one");
  pros::delay(500);
  turn(-45,600);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1, "two");
 
  pros::delay(500);
  move(13, 600);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                              "three");
 pros::delay(700);
  turn(95, 600);
  pros::delay(600);
  left_motors.move_velocity(200);
  right_motors.move_velocity(200);
  left_motors.set_voltage_limit(4600);
  right_motors.set_voltage_limit(4600);
  pros::delay(800);
  
    left_motors.move_velocity(0);
  right_motors.move_velocity(0);
    left_motors.set_voltage_limit(12000);
  right_motors.set_voltage_limit(12000);
  /*bool t;
  while (t==false) {
  left_motors.move_velocity(200);
  right_motors.move_velocity(200);
  
 if (front_left_motor.get_voltage()>6000 && front_right_motor.get_voltage()>6000) {
    left_motors.move_velocity(0);
  right_motors.move_velocity(0);
  t=true;
 }
  }*/
   

  pros::delay(500);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                              "four");
//  action.give();
//  }
//};

 
 pros::delay(400);

  //action.take(1500);
  snarf_rotator_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	snarf_rotator_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  snarf_rotator.move_absolute(-175, 200);
  pros::delay(400);
  snarf_driver.move_velocity(600);
  pros::delay(1000);
  snarf_driver.move_velocity(0);
  pros::delay(400);
  snarf_rotator.move_absolute(0,200);
  snarf_rotator_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	snarf_rotator_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //action.give();



  move(-4, 600);
  pros::delay(400);
  turn(270, 600);
   pros::delay(1200);
    move(30, 600);
   pros::delay(800);
  turn(-40, 600);
 pros::delay(300);
     left_motors.set_voltage_limit(5000);
  right_motors.set_voltage_limit(5000);

  move(36, 250);
  front_left_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	back_left_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	front_right_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  back_right_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	catapult_motor.set_brake_modes(MOTOR_BRAKE_HOLD);
     left_motors.set_voltage_limit(12000);
  right_motors.set_voltage_limit(12000);

}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
      left_motors.set_voltage_limit(12000);
  right_motors.set_voltage_limit(12000);

	//drive motors and cata motors set to actively hold position
	front_left_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	back_left_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	front_right_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  back_right_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	catapult_motor.set_brake_modes(MOTOR_BRAKE_HOLD);
	snarf_rotator_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	snarf_rotator_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//gets the x and y inputs from both controller sticks
        
        while (true) {
		    float left_y = (master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
        float right_y = (master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
        float left_x = (master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        float right_x = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
          pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                              "Power values: %f, %f, %f, %f", left_y, left_x,
                              right_y, right_x);
        
          /*if (left_x >= 5) {
            left_motors.move_velocity(left_x * modifier * left_y);
            if (0 <= left_x && left_x <= rt2 * 127) {
              right_motors.move_velocity(left_x / (rt2 * 127) * modifier *
                                         left_y);
            } else if (rt2 * 127 <= left_x && left_x <= 127.0) {
              right_motors.move_velocity(left_x / (rt2 * 127) * modifier *
                                         left_y * -1);
            }
            pros::delay(20);
          }
          if (left_x <= -5) {
            right_motors.move_velocity(left_x * modifier * left_y);
            if (-1 * rt2 * 127 <= left_x && left_x <= 0) {
              left_motors.move_velocity(left_x / (rt2 * 127) * modifier *
                                        left_y);
            } else if (-127.0 <= left_x && left_x <= -1 * rt2 * 127) {
              left_motors.move_velocity(left_x / (rt2 * 127) * modifier *
                                        left_y * -1);
            }
            pros::delay(20);
          }
          if (left_y>=5 && std::abs(left_x)<5){
            left_motors.move_velocity(modifier * left_y);
            right_motors.move_velocity( modifier * left_y);
          }
          if (left_y<=5 && std::abs(left_x)<5) {
             right_motors.move_velocity( modifier * left_y);
             left_motors.move_velocity( modifier * left_y);
          }
          if (std::abs(left_x)<5 && std::abs(left_y)<5){right_motors.move_velocity(0);
                left_motors.move_velocity(0);
          }*/
if (drive_plus_turning) { // Option to use the right stick for turning
      if (abs(right_x) < abs(left_x)) {
        right_x = left_x;
      }
    }
    if (drive_plus_forward) { // Option to use the right stick for forward
      if (abs(right_y) < abs(left_y)) {
        right_y = left_y;
      }
    }

    // Drive Control Loop (LEFT)
    y_direction = sgn(left_y);
    x_direction = sgn(right_x);

    float y_goal = powf((abs(left_y) / 127), 2) * 127;
    float x_goal = powf((abs(right_x) / 127), 2) * 127;
    
    float y_error = y_goal - y_current;
    float x_error = x_goal - x_current;

    if (((up_step > y_error) && (y_error > 0)) || ((down_step < y_error) && (y_error <= 0))) {
      y_true_step = y_error;
    } else if (y_goal > y_current) {
      y_true_step = up_step;
    } else if (y_goal < y_current) {
      y_true_step = down_step;
    } else {
      y_true_step = 0;
    }
    if (((up_step > x_error) && (x_error > 0)) || ((down_step < x_error) && (x_error <= 0))) {
      x_true_step = x_error;
    } else if (x_goal > x_current) {
      x_true_step = up_step;
    } else if (x_goal < x_current) {
      x_true_step = down_step;
    } else {
      x_true_step = 0;
    }
    y_current += y_true_step;
    x_current += x_true_step;

    if (standard_drive) {
      left_motors.move((y_current * y_direction) + (turn_constant * x_current * x_direction));
      right_motors.move((y_current * y_direction) - (turn_constant * x_current * x_direction));
    } else {
      left_motors.move((y_current * y_direction) - (turn_constant * x_current * x_direction));
      right_motors.move((y_current * y_direction) + (turn_constant * x_current * x_direction));
    }

    pros::delay(20);
  
         
          
          
          // snarfer code

          if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            snarf_driver.move_velocity(600);
          } 
          else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            snarf_driver.move_velocity(-600);
          }
          else {
            snarf_driver.move_velocity(0);
          }
       

          // forebar code- if not working reverse input and output

          if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (isextended == true) {
              snarf_rotator.move_absolute(0, 127);
              isextended = false;
              pros::delay(20);
            } 
            else if (isextended == false) {
              snarf_rotator.move_absolute(-extenddistance, 127);
              isextended = true;
              pros::delay(20);
            }

            else {
              pros::delay(20);
            }
          }

          // catapult code

if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && isextended==true) {
   catapult_motor.move_relative(540, 127);
  }
          pros::delay(20);	
if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
   catapult_motor.move_relative(45 , 127);
}
if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
   catapult_motor.move_relative(-45 , 127);}
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			piston.set_value(true);
		}
    
}
}