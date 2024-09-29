#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <string>

enum class Direction {
	LEFT, 
	RIGHT
};

pros::Motor_Group IntakeSpin({-6, 7});
pros::Motor_Group ArmRotate({-15, 16});
pros::Motor ArmExtend (2, pros::E_MOTOR_GEAR_RED);
pros::Motor_Group ParkingStick ({-1, 10});
pros::Motor WingSlapper(3);
pros::ADIDigitalOut Wings ('A');
bool Recenter;


class DriveTrain
{
public:
    DriveTrain() : 
		FrontRight ({19, 20}),
		FrontLeft ({-11, -12}),
		RearRight ({17, 18}),
		RearLeft ({-13, -14}),
		TinyBox(21)
		{	FrontRight.set_gearing(pros::E_MOTOR_GEAR_200);
			FrontLeft.set_gearing(pros::E_MOTOR_GEAR_200);
			RearRight.set_gearing(pros::E_MOTOR_GEAR_200);
			RearLeft.set_gearing(pros::E_MOTOR_GEAR_200);
			};
	void setDrivetrainPower(double axF, double axS, double axT);
	void setDrivetrainVelocity(double speed);
	void setBrakeMode(pros::motor_brake_mode_e mode);
	void Drive_IN (double distance, double power);
	void Drive_TIME (double time, double power);
	void Strafe_IN(double distance, double power, Direction sd);
	void Strafe_TIME(double time, double power, Direction sd);
	void Turn_DEG (double distance, double power, Direction td);
	void Turn_LHD (double target, double power);
	void Turn_RHD (double target, double power);
	
	std::string getMotorPositionsF();
	std::string getMotorPositionsR();
	std::string getHeading();
	double getHeading2();
	void resetHeading();
	void setCurrentLimit( int newLimit);

private:
	pros::Motor_Group FrontRight;
	pros::Motor_Group FrontLeft;
	pros::Motor_Group RearRight;
	pros::Motor_Group RearLeft;
	pros::IMU TinyBox;
	const double _DriveMultiplier = (5350/74.25);
	const double _StrafeMultiplier = (2993/36);
	const double _TurnMultiplier = (6336/360);
};

void DriveTrain::setDrivetrainPower(double f, double s, double t){

	double motorPowers [4];
	motorPowers[0] = ((f-t-s)/127.0)*200;
	motorPowers[1] = ((f+t+s)/127.0)*200;
	motorPowers[2] = ((f-t+s)/127.0)*200;
	motorPowers[3] = ((f+t-s)/127.0)*200;
	
	FrontRight.move_velocity(motorPowers[0]);
	FrontLeft.move_velocity(motorPowers[1]);
	RearRight.move_velocity(motorPowers[2]);
	RearLeft.move_velocity(motorPowers[3]);
}

void DriveTrain::setDrivetrainVelocity(double speed){
	FrontRight.move_velocity(speed);
	FrontLeft.move_velocity(speed);
	RearRight.move_velocity(speed);
	RearLeft.move_velocity(speed);
}

void DriveTrain::setBrakeMode(pros::motor_brake_mode_e mode){
	FrontRight.set_brake_modes(mode);
	FrontLeft.set_brake_modes(mode);
	RearRight.set_brake_modes(mode);
	RearLeft.set_brake_modes(mode);
}

void DriveTrain::Drive_IN(double distance, double power){
	FrontRight.tare_position();
	FrontLeft.tare_position();
	RearRight.tare_position();
	RearLeft.tare_position();
	double target =  distance * _DriveMultiplier;
	FrontRight.move_relative(target, power * 2);
	FrontLeft.move_relative(target, power * 2);
	RearRight.move_relative(target, power * 2);
	RearLeft.move_relative(target, power * 2);
	pros::delay(1000);
	while (FrontLeft.get_actual_velocities()[0] > 0 || FrontRight.get_actual_velocities()[0] > 0 || RearLeft.get_actual_velocities()[0] > 0 || RearRight.get_actual_velocities()[0] > 0) 
	{}
}

void DriveTrain::Drive_TIME(double time, double power){
	FrontRight = power;
	FrontLeft = power;
	RearRight = power;
	RearLeft = power;
	pros::delay(time);
	FrontRight = 0;
	FrontLeft = 0;
	RearRight = 0;
	RearLeft = 0;
}

void DriveTrain::Strafe_IN(double distance, double power, Direction sd){
	FrontRight.tare_position();
	FrontLeft.tare_position();
	RearRight.tare_position();
	RearLeft.tare_position();
	double target =  distance * _StrafeMultiplier;
	if (sd == Direction::LEFT){
		target *= -1;
	}
	FrontRight.move_relative(-target, power * 2);
	FrontLeft.move_relative(target, power * 2);
	RearRight.move_relative(target, power * 2);
	RearLeft.move_relative(-target, power * 2);
	pros::delay(1000);
	while (FrontLeft.get_actual_velocities()[0] > 0 || FrontRight.get_actual_velocities()[0] > 0 || RearLeft.get_actual_velocities()[0] > 0 || RearRight.get_actual_velocities()[0] > 0) 
	{}
}

void DriveTrain::Strafe_TIME(double time, double power, Direction sd){
	if (sd == Direction::LEFT){
		power *= -1;
	}

	FrontRight = -power;
	FrontLeft = power;
	RearRight = power;
	RearLeft = -power;
	pros::delay(time);
	FrontRight = 0;
	FrontLeft = 0;
	RearRight = 0;
	RearLeft = 0;
}

void DriveTrain::Turn_DEG(double distance, double power, Direction td){
	FrontRight.tare_position();
	FrontLeft.tare_position();
	RearRight.tare_position();
	RearLeft.tare_position();
	double target =  distance * _TurnMultiplier;
	if (td == Direction::LEFT){
		target *= -1;
	}
	FrontRight.move_relative(-target, power * 2);
	FrontLeft.move_relative(target, power * 2);
	RearRight.move_relative(-target, power * 2);
	RearLeft.move_relative(target, power * 2);
	pros::delay(1000);
	while (FrontLeft.get_actual_velocities()[0] > 0 || FrontRight.get_actual_velocities()[0] > 0 || RearLeft.get_actual_velocities()[0] > 0 || RearRight.get_actual_velocities()[0] > 0) 
	{}
}

void DriveTrain::Turn_RHD(double target, double power){
	double startHeading = TinyBox.get_heading();
	double offset = 0;
	while (startHeading + offset > target){
		offset -= 360;
	}
	FrontRight.move_velocity(-power * 2);
	FrontLeft.move_velocity(power * 2);
	RearRight.move_velocity(-power * 2);
	RearLeft.move_velocity(power * 2);
	pros::lcd::set_text(1, std::to_string(startHeading));
	pros::lcd::set_text(2, std::to_string(offset));
	pros::lcd::set_text(3, std::to_string(startHeading + offset));
	
	while (TinyBox.get_heading() + offset < target){
		if (TinyBox.get_heading() < target){
			offset = 0;
		}
		pros::lcd::set_text(4, std::to_string(TinyBox.get_heading() + offset));
	}
	pros::lcd::set_text(5, "GOT TO 5");
	FrontRight.brake();
	FrontLeft.brake();
	RearRight.brake();
	RearLeft.brake();


}

void DriveTrain::Turn_LHD(double target, double power){
	double startHeading = TinyBox.get_heading();
	double offset = 0;
	while (startHeading + offset < target){
		offset += 360;
	}
	FrontRight.move_velocity(power * 2);
	FrontLeft.move_velocity(-power * 2);
	RearRight.move_velocity(power * 2);
	RearLeft.move_velocity(-power * 2);
	pros::lcd::set_text(1, std::to_string(startHeading));
	pros::lcd::set_text(2, std::to_string(offset));
	pros::lcd::set_text(3, std::to_string(startHeading + offset));
	
	while (TinyBox.get_heading() + offset > target){
		if (TinyBox.get_heading() > target){
			offset = 0;
		}
		pros::lcd::set_text(4, std::to_string(TinyBox.get_heading() + offset));
	}
	pros::lcd::set_text(5, "GOT TO 5");
	FrontRight.brake();
	FrontLeft.brake();
	RearRight.brake();
	RearLeft.brake();

}


std::string DriveTrain::getMotorPositionsF(){
	return 
	"FL: " + std::to_string((int)FrontLeft.get_positions()[0]) + 
	", FR: " + std::to_string((int)FrontRight.get_positions()[0]);
}

std::string DriveTrain::getMotorPositionsR(){
	return 
	"RL: " + std::to_string((int)RearLeft.get_positions()[0]) + 
	", RR: " + std::to_string((int)RearRight.get_positions()[0]);
}

std::string DriveTrain::getHeading(){
	return "DT Heading: " + std::to_string(TinyBox.get_heading());
}

double DriveTrain::getHeading2(){
	return TinyBox.get_heading();
}

void DriveTrain::resetHeading(){
	TinyBox.reset(true);
}

void DriveTrain::setCurrentLimit(int newLimit){
FrontLeft.at(0).set_current_limit(newLimit);
FrontLeft.at(1).set_current_limit(newLimit);
RearLeft.at(0).set_current_limit(newLimit);
RearLeft.at(1).set_current_limit(newLimit);
FrontRight.at(0).set_current_limit(newLimit);
FrontRight.at(1).set_current_limit(newLimit);
RearRight.at(0).set_current_limit(newLimit);
RearRight.at(1).set_current_limit(newLimit);
}


DriveTrain DT;
	


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	Wings.set_value(false);
	DT.resetHeading();
	//IntakeSpin[0].set_current_limit(1200);
	ParkingStick.tare_position();
	ParkingStick[0].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	ParkingStick[1].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Recenter = false;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	WingSlapper.tare_position();
	for(int i = 0; i < 24; i++){
		WingSlapper = 127;
		pros::delay(400);
		WingSlapper.move_absolute(0, 200);
		pros::delay(1500);
	}
	Wings.set_value(true);
	pros::delay(200);
	DT.Drive_IN(7, 100);
	DT.Turn_DEG(50, 50, Direction::RIGHT);
	DT.Drive_IN(5, 100);
	WingSlapper = 127;
	pros::delay(400);
	WingSlapper.move_absolute(0, 200);
	pros::delay(1500);
	DT.Turn_DEG(8, 100, Direction::LEFT);
	Wings.set_value(false);
	DT.Drive_IN(70, 100);
	DT.Drive_IN(-20, 100);

	return;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
	//Hang.move_absolute(-3000, 200);
	uint32_t timeSinceStart = pros::millis();
	bool hangDeploy2 = false;
	bool hangOverride = false;
	bool hangBuzzer = false;
	bool intakeLock = false;
	bool wingLock = false;
	WingSlapper.tare_position();

	ParkingStick.move_absolute(0, 200);
	if (!Recenter){
	ArmExtend.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	ArmRotate.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	ArmRotate.move_velocity(-100);
	ArmExtend.move_velocity(-100);
	pros::c::delay(700);
	ArmRotate.move_velocity(0);
	ArmExtend.move_velocity(0);
	ArmRotate.tare_position();
	ArmExtend.tare_position(); 
	}

	pros::Controller ControllerMain(pros::E_CONTROLLER_MASTER);
	DT.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	
	bool trip = false;
	bool ninput;
	int extendzero = 0;
	Wings.set_value(false);
	int update = 0;
	


	while (true) {
		
		// pros::lcd::set_text(1, std::to_string(ArmExtend.get_position()));
		// pros::lcd::set_text(2, std::to_string(ArmRotate.get_positions()[0]));
		// pros::lcd::set_text(3, std::to_string(extendzero));
		if (update <= 0){
		pros::lcd::set_text(1, DT.getMotorPositionsF());	
		pros::lcd::set_text(2, DT.getMotorPositionsR());
		pros::lcd::set_text(3, DT.getHeading());
		//pros::lcd::set_text(4, std::to_string(Hang.get_positions()[0]));
		update = 100;
		}
		update--;
		
		double f = ControllerMain.get_analog(ANALOG_LEFT_Y);
		double s = ControllerMain.get_analog(ANALOG_LEFT_X);
		double t = ControllerMain.get_analog(ANALOG_RIGHT_X);
		
		DT.setDrivetrainPower(f, s, t);

		// double motorPowers [4];
		// motorPowers[0] = ((f-t-s)/127.0)*200;
		// motorPowers[1] = ((f+t+s)/127.0)*200;
		// motorPowers[2] = ((f-t+s)/127.0)*200;
		// motorPowers[3] = ((f+t-s)/127.0)*200;

		// FrontRight = motorPowers[0];
		// FrontLeft = motorPowers[1];
		// RearRight = motorPowers[2];
		// RearLeft = motorPowers[3];

		if (ControllerMain.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			IntakeSpin = 127;
		} else if (ControllerMain.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			IntakeSpin = -127;
		} else {
			IntakeSpin = 0;
		}

		if(ControllerMain.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			ArmRotate=127;
			// extendzero = 300;
		} else if (ControllerMain.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			ArmRotate=-127;
			ninput = false;
			extendzero = 0;
		} else {
			ninput = true;
			ArmRotate = 0;
		}

		if (std::abs(ArmRotate.get_positions()[0]) > 1000 && ninput){
			extendzero = 0;
		} else {
			if(std::abs(ArmRotate.get_actual_velocities()[0]) < 1){
			extendzero = 0;
			}
		}

		if(ControllerMain.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && !wingLock){
			ArmExtend.move_absolute(1150, 50);
			intakeLock = true;
		} else {
			if (std::abs(ArmExtend.get_position() - extendzero) > 50){
			ArmExtend.move_absolute(extendzero, 100);
			} else {
			ArmExtend = 0;
			}
			intakeLock = false;
		}

		if (ControllerMain.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && !intakeLock){
			Wings.set_value(true);
			wingLock = true;
		} else {
			Wings.set_value(false);
			wingLock = false;
		}


		if (ControllerMain.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			WingSlapper = 127;
		} else {
			WingSlapper.move_absolute(0, 200);
		}

		

		pros::delay(20);
	}
}
