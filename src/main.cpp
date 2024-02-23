
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
using pros::E_CONTROLLER_ANALOG_LEFT_X;
using pros::E_CONTROLLER_ANALOG_RIGHT_X;
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
bool autoncheck;
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

pros::Motor front_left_motor(16,pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right_motor(20,pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left_motor(17,pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right_motor(19,pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor catapult_motor_left(1, pros::E_MOTOR_GEAR_RED, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor catapult_motor_right(2, pros::E_MOTOR_GEAR_RED, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor snarf_rotator_right( 4,pros::E_MOTOR_GEAR_GREEN, false, pros:: E_MOTOR_ENCODER_DEGREES );
pros::Motor snarf_rotator_left( 19,pros::E_MOTOR_GEAR_GREEN, true, pros:: E_MOTOR_ENCODER_DEGREES );
pros::Motor snarf_driver(11, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
//grouping both sides of the drivetrain
pros::MotorGroup left_motors({front_left_motor, back_left_motor});
pros::MotorGroup right_motors({front_right_motor, back_right_motor});
pros::MotorGroup snarf_rotator({snarf_rotator_right, snarf_rotator_left});
pros::MotorGroup catapult_motor({catapult_motor_left, catapult_motor_right});
int repeat=0;
//modifier from 127 to 600
float modifier = 600.0/127;
//just the square root of 2
float rt2 = sqrt(2.0);
//defining the controller
int auton_step = 0; // debug
int catainit =120; // side one is defensive, side -1 is offensive
float autonselect=   1;
void debug_auton(int step){
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "Running Auton");
  pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Step: %d", step);
}

float extenddistance = 250;
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
void snarf_turn(){
            if (isextended == true) {
              snarf_rotator.move_relative(extenddistance, 127);
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


pros::Mutex action;
// inches, rpm, defaults to forward
void move( float inches, float velocity) { 	//action.take(1000);
	Forward_Wheel_Rotation = Move_Tuning_Factor*(inches/Wheel_Circumference)*360;

  left_motors.tare_position();
  right_motors.tare_position();

  left_motors.move_relative(Forward_Wheel_Rotation,velocity);
  right_motors.move_relative(Forward_Wheel_Rotation,velocity);
		//action.give();
	}
  // inputs in degrees, rpm, defaults to clockwise
void turn (float angle, float velocity) {
	//action.take(1000);
	 Turning_Distance = angle/360 * Turning_Circumference;
  Wheel_Revolutions = Turning_Distance/Wheel_Circumference;
  Turn_Wheel_Rotation = Turn_Tuning_Factor*Wheel_Revolutions*360; // Degrees that the wheel should turn
  left_motors.move_relative(Turn_Wheel_Rotation*autonselect, velocity);
  right_motors.move_relative(-Turn_Wheel_Rotation*autonselect, velocity);

}
void timedmove (float inches, float velocity, float milliseconds) {
 
  Forward_Wheel_Rotation = Move_Tuning_Factor*(inches/Wheel_Circumference)*360;
float distance;
float time;
  while (not (Forward_Wheel_Rotation <= distance or time <= milliseconds)) {
    left_motors.move_velocity(velocity);
    right_motors.move_velocity(velocity);


    pros::delay(10);
    time= time+10;
    distance = Move_Tuning_Factor*Wheel_Circumference*velocity*time/60000;


  }
  

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
  autoncheck=true;
  catapult_motor.set_brake_modes(MOTOR_BRAKE_HOLD);
  	snarf_rotator_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	snarf_rotator_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 
move(-30, 400);
pros::delay(800);
turn(50, 600);

pros::delay(500);
  left_motors.move_velocity(-600);
  right_motors.move_velocity(-600);
  pros::delay(700);
 left_motors.move_velocity(0);
  right_motors.move_velocity(0);
//pros::Task body1{ 
//  [=]{
  //  action.take(5000);
  move(4, 600);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                              "one");
  pros::delay(500);
  
  turn(-45,500);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1, "two");
 
  pros::delay(600);
  //moves to centre of elevation bar
  move(17, 600);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                              "three");
 pros::delay(700);
 //tweak this turn as needed at tournament -aligns with match load bar
  turn(105, 600);
  pros::delay(500);
  left_motors.move_velocity(300);
  right_motors.move_velocity(300);
  left_motors.set_voltage_limit(4000);
  right_motors.set_voltage_limit(4000);
  pros::delay(900);
  
    left_motors.move_velocity(0);
  right_motors.move_velocity(0);
    left_motors.set_voltage_limit(12000);
  right_motors.set_voltage_limit(12000);
  pros::delay(600);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1, "four");
  snarf_turn(); 
  pros::delay(600);
  catapult_motor.move_relative(235 , 127);
  pros::delay(500);
  snarf_driver.move_velocity(600);
  pros::delay(700);
  	snarf_rotator_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	snarf_rotator_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   move(-6, 600);
   pros::delay(800);
if(autonselect==1) {
  turn(25, 400);
   pros::delay(1200);
   catapult_motor.move_relative(-45,127);
   pros::delay(300);
   catapult_motor.move_relative(200, 127);
  pros::delay(800);
  //tweak this turn and move at tourney- supposed to get robot into position where the following turn will allow the snarfer to be aligned with the center triball
   turn(-135, 400);
      pros::delay(800);
    snarf_turn();
   snarf_driver.move_velocity(0);
   pros::delay(500);
    move(23, 600);
   pros::delay(600);
  turn(-40, 600);
   pros::delay(500);
   snarf_turn();
   //go forward and touch the bar
   snarf_driver.move_velocity(-600);
  move(20,600);
  pros::delay(600);

  
     left_motors.set_voltage_limit(5000);
  right_motors.set_voltage_limit(5000);

  move(5, 250);
    }
    //12.7 seconds 
else if (autonselect==-1) {
  move(-6, 600);
    turn(-25, 400);
   pros::delay(1000);
   catapult_motor.move_relative(540, 127);
}
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
	snarf_rotator_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	snarf_rotator_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 if (autoncheck) {catapult_motor.move_relative(370, 200);
 pros::delay(500);
}
     snarf_driver.move_velocity(0);
	//gets the x and y inputs from both controller sticks
        
      
        while (true) {
		    float left_y = (master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
        float right_y = (master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
        float left_x = (master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        float right_x = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
          pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                              "Power values: %f, %f, %f, %f", left_y, left_x,
                              right_y, right_x);
        
        
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

      if (drive_plus_turning) { // Option to use the right stick for turning
      if (abs(right_y) < abs(left_y)) {
        right_y = left_y;
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
      right_motors.move(((y_current * y_direction) - (turn_constant * x_current * x_direction))/1.005);
    } else {
      left_motors.move((y_current * y_direction) - (turn_constant * x_current * x_direction));
      right_motors.move(((y_current * y_direction) + (turn_constant * x_current * x_direction))/1.005);
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
      snarf_turn();
          }
          // catapult code

if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
   catapult_motor.move_relative(540, 127); //pos was 540
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
if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
   catapult_motor.move_relative(catainit , 127);
}
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
    snarf_rotator.move_absolute(0, 200);
  }
}
}