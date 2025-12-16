#include "main.h"
#include <cmath>
#include <cstdio>

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({1, 2, 3});
pros::MotorGroup right_mg({-4, -5, -6});

pros::Motor Hood(7);
pros::Motor Storage(8);
pros::Motor Top(9);
pros::Motor Front(10);

pros::Imu inertial(11);
pros::Rotation xrot(12);
pros::Rotation yrot(13);
pros::ADIDigitalOut ML('A');

static double botx = 0.0; //placeholer values (need do decide start position)
static double boty = 0.0;
static double both = 0.0;

// A = left side of the park zone in the back
const int startAx = 0; //need to test dis is placeholder )
const int startAy = 0;
const int startAang = 0;

// B = left side of the park zone in the front
const int startBx = 0; //need to test dis is placeholder )
const int startBy = 0;
const int startBang = 0;

// C = right side of the park zone in the fornt
const int startCx = 0; //need to test dis is placeholder )
const int startCy = 0;
const int startCang = 0;

// D = right side of the park zone in the back
const int startDx = 0; //need to test dis is placeholder )
const int startDy = 0;
const int startDang = 0;

// middle goal is (0,0)
// heading 0 is facing perpendicular to field wall

const int ekp = 0;// need to tune dis
const int ekd = 0;

double saved_drive_error = 0.0;

double inith = 0.0;
double initv = 0.0;

double rad2deg = 180.0 / acos(-1.0);

double adjustment = .01; // need to tune dis
void updatepose(void*){
	double relxy = ((xrot.get_position() * rad2deg / 180.0)- inith) + ((yrot.get_position() * rad2deg / 180.0) - initv);
	boty += (relxy) * sin(both);
	botx += (relxy) * cos(both);
	double inith = xrot.get_position() * rad2deg / 180.0;
	double initv = yrot.get_position() * rad2deg / 180.0;

	double both = inertial.get_heading();

	printf("X: %.2f Y: %.2f H: %.2f\n", botx, boty, both);

	(void)boty;
	(void)botx;
	(void)both;
}

void initialize() {
	left_mg.move_velocity(999);
	right_mg.move_velocity(999);
	Hood.move_velocity(999);
	Top.move_velocity(999);
	Front.move_velocity(999);
	Storage.move_velocity(999);
	inertial.reset();
	if(inertial.is_calibrating()){
		printf("imu_sensor is calibrating");
	}
	xrot.reset();
	yrot.reset();

	pros::Task odometry_task_pose(updatepose, nullptr);
}

void disabled() {
}
void competition_initialize() {
	initialize();
}


static double vperc(double perc) {
	return (perc / 100.0) * 200.0;
}

void score_bottom(int vel, bool yn) {
	if (yn) {
		Front.move_velocity(-vperc(vel));
		Storage.move_velocity(-vperc(vel));
	} else {
		Front.move_velocity(0);
		Storage.move_velocity(0);
	}
}

void score_top(int vel, bool yn) {
	if (yn) {
		Front.move_velocity(vperc(vel));
		Storage.move_velocity(-vperc(vel));
		Hood.move_velocity(vperc(vel));
		Top.move_velocity(vperc(vel));
	} else {
		Front.move_velocity(0);
		Storage.move_velocity(0);
		Hood.move_velocity(0);
		Top.move_velocity(0);
	}
}

void score_mid(int vel, bool yn) {
	if (yn) {
		Front.move_velocity(vperc(vel));
		Storage.move_velocity(-vperc(vel));
		Top.move_velocity(-vperc(vel));
	} else {
		Front.move_velocity(0);
		Storage.move_velocity(0);
		Top.move_velocity(0);
	}
}

void intake(int vel, bool yn) {
	if (yn) {
		Front.move_velocity(vperc(vel));
		Storage.move_velocity(-vperc(vel));
	} else {
		Front.move_velocity(0);
		Storage.move_velocity(0);
	}
}

void almost_top(int vel, bool yn) {
	if (yn) {
		Front.move_velocity(vperc(vel));
		Storage.move_velocity(-vperc(vel));
		Hood.move_velocity(-vperc(vel));
		Top.move_velocity(vperc(vel));
	} else {
		Front.move_velocity(0);
		Storage.move_velocity(0);
		Hood.move_velocity(0);
		Top.move_velocity(0);
	}
}

void opcontrol() {
	double speed = 1;
	while (true){
		left_mg.move_velocity(vperc(((((pow(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2)*adjustment) *-1 - pow(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 2)*adjustment))) * -speed));
		right_mg.move_velocity(vperc(((((pow(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2)*adjustment) *-1 + pow(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 2)*adjustment))) * -speed));

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			score_bottom(100, true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake(100, true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			score_top(100, true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			score_mid(100, true);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			double speed = 1.0;
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			double speed = 0.3;
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			ML.set_value(false);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			ML.set_value(true);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			almost_top(100, true);
		} else {
			almost_top(100, false);
			score_bottom(100, false);
			score_mid(100, false);
			intake(100, false);
			score_top(100, false);
		}
		pros::delay(20);
	}
}


//auton

void autonomous() {

}

void safeauton() {

}

void midauton() {

}

void riskauton() {

}

void sauton() {

}


//odometry + PID :(
void turn(double ang, double kd, double kp) {
	double direction = 0;
	double old_error = 0;
	double Raw_angle = inertial.get_heading();
	double shead = Raw_angle;
	int is_done = 0;
	double derivative = 0;
	double gyro_rate = 0;
	while (Raw_angle > 180) { 
		Raw_angle -= 360;
	}
	while (Raw_angle < -180) {
		Raw_angle += 360;
	}
	double error = ang - Raw_angle;
	while (is_done < 10){

		if (error > -0.01 && error < 0.01){
			is_done = is_done + 1;
		} else {
			is_done = 0;
		}

		Raw_angle = inertial.get_heading();
		while (Raw_angle > 180) { 
			Raw_angle -= 360;
		}
		while (Raw_angle < -180) {
			Raw_angle += 360;
		}
		error = ang - Raw_angle;

		gyro_rate = inertial.get_gyro_rate().z; // degrees per second
        derivative = -gyro_rate; // negative in order to slow down the bot
		double tvel = kp * error + kd * derivative;

		if (shead < 0 && ang < 0) {
			if (ang < shead) {
				double direction = -1;
			}
			if (ang > shead) {
				double direction = 1;
			}
		}
		else if (shead > 0 && ang > 0) {
			if (ang < shead) {
				double direction = 1;
			}
			if (ang > shead) {
				double direction = -1;
			}
		}
		else if (shead > 0 && ang < 0) {
			if (ang < -1*shead) {
				double direction = -1;
			}
			if (ang > -1*shead) {
				double direction = 1;
			}
		}
		else if (shead < 0 && ang > 0) {
			if (ang < -1*shead) {
				double direction = 1;
			}
			if (ang > -1*shead) {
				double direction = -1;
			}
		}

		left_mg.move_velocity(direction*(vperc(tvel)));
		right_mg.move_velocity(direction*(vperc(tvel)));

		pros::delay(20);
		old_error = error;
	}
	right_mg.move_velocity(0);
	left_mg.move_velocity(0);
}

double drive_error_correction(double kd, double kp, double error) {
	double old_error = saved_drive_error;
	double new_error = error;
	double derivative = (new_error - old_error)/0.02;
	double tvel = error * kp + kd * derivative;
	saved_drive_error = new_error;
	return tvel;
}

double abs2(double value) {
	if (value < 0) {
		value = -1 * value;
	}
	return value;
}

void drive(double dis, double kd, double kp, double ekd, double ekp) {
	double relerror = 0;
	double drive_correction = 0;
	double raw_dis = yrot.get_position();
	double raw_xerror = xrot.get_position();
	double sdis = raw_dis;
	int is_done = 0;
	int direction = 0;
	double distance_derivative_1 = 0;
	double distance_derivative_2 = 0;
	double distance_derivative_3 = 0;
	double smoothed_distance_derivative = 0;
	double correction_check = 0;
	while (sdis > 180) { 
		sdis -= 360;
	}
	while (sdis < -180) {
		sdis += 360;
	}
	double error = dis - sdis;
	double old_error = error;
	while (is_done < 10) {
		if (fabs(error) > 0.01 && fabs(error) < 0.1) {
			is_done += 1;
		} else {
			is_done = 0;
		}


		if (xrot.get_position() > 0 && raw_xerror > 0) {
			double relerror = (abs2(xrot.get_position()) - abs2(raw_xerror)) * 2;
		}
		else if (xrot.get_position() < 0 && raw_xerror > 0) {
			double relerror = (abs2(xrot.get_position()) - raw_xerror) * 2;
		}
		else if (xrot.get_position() > 0 && raw_xerror < 0) {
			double relerror = (xrot.get_position() - abs2(raw_xerror)) * 2;
			}
		else if (xrot.get_position() < 0 && raw_xerror < 0) {
			double relerror = (abs2(xrot.get_position()) - abs2(raw_xerror)) * 2;
		}

		if (relerror < .1) {
			double drive_correction = drive_error_correction(ekd, ekp, relerror);
		}
		else {
			double drive_correction = 0;
			double saved_drive_error = 0;
		}

		raw_dis = yrot.get_position();
		while (raw_dis > 180) { 
			raw_dis -= 360;
		}
		while (raw_dis < -180) {
			raw_dis += 360;
		}
		error = dis - raw_dis;

		distance_derivative_3 = distance_derivative_2;
        distance_derivative_2 = distance_derivative_1;
        distance_derivative_1 = (error - old_error)/0.02;
        smoothed_distance_derivative = (distance_derivative_1 + distance_derivative_2 + distance_derivative_3)/3;
		double tvel = error * kp + kd * smoothed_distance_derivative + 10;

		if (sdis < 0 && dis < 0) {
			if (dis < sdis) {
				double direction = -1;
			}
			if (dis > sdis) {
				double direction = 1;
			}
		}
		else if (sdis > 0 && dis > 0) {
			if (dis < sdis) {
				double direction = 1;
			}
			if (dis > sdis) {
				double direction = -1;
			}
		}
		else if (sdis > 0 && dis < 0) {
			if (dis < -1*sdis) {
				double direction = -1;
			}
			if (dis > -1*sdis) {
				double direction = 1;
			}
		}
		else if (sdis < 0 && dis > 0) {
			if (dis < -1*sdis) {
				double direction = 1;
			}
			if (dis > -1*sdis) {
				double direction = -1;
			}
		}

		if (xrot.get_position() < 0 && raw_xerror > 0) {
			double correction_check = (abs2(xrot.get_position()) - raw_xerror) * 2;
		}
		else if (xrot.get_position() > 0 && raw_xerror < 0) {
			double correction_check = (xrot.get_position() + raw_xerror) * 2;
			}
		else {
			double correction_check = (xrot.get_position() - raw_xerror) * 2;
		}


		if (correction_check > 0) {
			left_mg.move_velocity(direction * vperc(fabs(tvel + drive_correction)));
			right_mg.move_velocity(direction * vperc(fabs(tvel - drive_correction))); // will need to tune the +/- here
		}
		else {
			left_mg.move_velocity(direction * vperc(fabs(tvel - drive_correction)));
			right_mg.move_velocity(direction * vperc(fabs(tvel + drive_correction)));
		} 

		pros::delay(20);

		old_error = error;
	}
	
}

void cordon(double x, double y, double dkd, double dkp, double tkd, double tkp, double theta) {
	double difx = x - botx;
	double dify = y - boty;
	double desired_heading = std::atan2(dify, difx) * rad2deg;
	while (desired_heading > 180.0) desired_heading -= 360.0;
	while (desired_heading < -180.0) desired_heading += 360.0;
	double drive_distance = std::hypot(difx, dify);
	turn(desired_heading, tkd, tkp);
	pros::delay(100);
	drive(drive_distance, dkd, dkp, ekp, ekd);
	if (theta != 0) {
		turn(theta, tkd, tkp);
	}
}