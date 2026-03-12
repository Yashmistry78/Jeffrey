#include "main.h"
#include "lemlib/api.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, -12, 13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-18, 19, 20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(14);

pros::Motor intakeMotor1(10, pros::MotorGearset::blue);
pros::Motor intakeMotor2(17, pros::MotorGearset::blue);

pros::ADIDigitalOut intakePiston('A');
pros::ADIDigitalOut unloaderPiston('E');
pros::ADIDigitalOut descorePiston('D');
pros::ADIDigitalOut middescorePiston('B');

bool intakeOn = false;
bool intakeReverseOn = false;
bool bottomOnlyOn = false;
bool pistonOn = false;

bool intakePistonState = false;
bool pistonCState = false;
bool pistonBState = false;
bool pistonFstate = false;

// Button edge tracking
bool r1Last = false;
bool r2Last = false;
bool l1Last = false;
bool aLast = false;
bool upLast = false;
bool xLast = false;
bool rightLast = false;

//color sensor
bool enableParkDetect = false;
pros::Optical parkSensor(15);

//anti jam
bool intakeRunning = false;
bool antiJamEnabled = false;

//distance sensors
pros::Distance leftSensor(1);
pros::Distance rightSensor(8);
pros::Distance backLeftSensor(2);
pros::Distance backRightSensor(9);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(16);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.25);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(9, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            46, // derivative gain (kD)
                                            0, // anti windup
                                            1.5, // small error range, in inches
                                            150, // small error range timeout, in milliseconds
                                            5, // large error range, in inches
                                            1500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.9, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             14.25, // derivative gain (kD)
                                             0, // anti windup
                                             0.5, // small error range, in degrees
                                             150, // small error range timeout, in milliseconds
                                             5, // large error range, in degrees
                                             1500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(1, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(1, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

bool onPark() {
    parkSensor.set_led_pwm(100);

    double hue = parkSensor.get_hue();
    double bright = parkSensor.get_brightness();

    // red wraps around hue circle
    if (hue < 25 || hue > 315) return true;

    return false;
}

void parkDetectorTask(void*) {
    while (true) {

        if (enableParkDetect && onPark()) {

            // stop LemLib motion controller
            chassis.cancelMotion();
            unloaderPiston.set_value(false);
        }

        pros::delay(10);
    }
}

pros::Task parkTask(parkDetectorTask);

void antiJamTaskFn(void*) {

    int jamTimer = 0;

    while(true) {

        if (antiJamEnabled && intakeRunning) {

            double vel = fabs(intakeMotor1.get_actual_velocity());

            if (vel < 10) {
                jamTimer += 10;
            } else {
                jamTimer = 0;
            }

            if (jamTimer > 120) {
                intakeMotor1.move(-96);
                intakeMotor2.move(96);
                pros::delay(200);

                intakeMotor1.move(127);
                intakeMotor2.move(-127);

                jamTimer = 0;
            }

        }    

        pros::delay(10);
    }
}

pros::Task antiJamTask(antiJamTaskFn);


void waitUntilIntakeDone(int timeout = 2500) {

    int time = 0;
    int stableTime = 0;

    while(time < timeout) {

        double vel = fabs(intakeMotor1.get_actual_velocity());

        // still scoring (loaded)
        if(vel < 650) {
            stableTime = 0;
        }

        // finished scoring (free spinning)
        else if(vel > 670) {
            stableTime += 10;
        }

        // require stable free-spin for 300ms
        if(stableTime > 300) {
            break;
        }

        pros::delay(10);
        time += 10;
    }
}



const double back_sensor_left_offset  = 5.46875;
const double back_sensor_right_offset = 5.46875;
const double back_sensor_spacing      = 10.65625;
const double left_sensor_offset       = 5.2187;
const double right_sensor_offset      = 5.2187;
const double field_half_size          = 72.0;

void resetPositionAndHeadingBack(pros::Distance& back_left=backLeftSensor, pros::Distance& back_right=backRightSensor,
                                  double sensor_spacing = back_sensor_spacing,
                                  double left_offset = back_sensor_left_offset,   double right_offset = back_sensor_right_offset,
                                  double field_half=72.0) {

    double d_left  = (back_left.get()+19)  / 25.4; // mm to inches
    double d_right = back_right.get() / 25.4;

    // Validate readings
    if (d_left < 0 || d_left > 200 || d_right < 0 || d_right > 200) {
        printf("Invalid back sensor readings: L=%.1f R=%.1f\n", d_left, d_right);
        return;
    }

    // calculate angle to wall from the two sensor readings
    // positive angle = robot is rotated clockwise from perpendicular
    double angle_to_wall_rad = atan2(d_right - d_left, sensor_spacing);
    double angle_to_wall_deg = angle_to_wall_rad * 180.0 / M_PI;

    // calculate corrected perpendicular distance
    double avg_offset   = (left_offset + right_offset) / 2.0;
    double avg_reading  = (d_left + d_right) / 2.0;
    double corrected_distance = avg_reading * cos(angle_to_wall_rad) + avg_offset;

    // determine which wall the BACK of the robot is facing
    // back of robot = heading + 180°
    lemlib::Pose pose = chassis.getPose();
    double back_heading_deg = pose.theta + 180.0;
    int headingDeg = ((int)back_heading_deg % 360 + 360) % 360;

    bool   resettingX = false;
    double wallSign   = 1.0;
    double expected_perpendicular_heading = 0.0;

    if (headingDeg >= 315 || headingDeg <= 45) {
        // back faces top wall → reset Y (positive side)
        resettingX = false;
        wallSign   = 1.0;
        expected_perpendicular_heading = 180.0;
    }
    else if (headingDeg > 45 && headingDeg <= 135) {
        // back faces right wall → reset X (positive side)
        resettingX = true;
        wallSign   = 1.0;
        expected_perpendicular_heading = 270.0;
    }
    else if (headingDeg > 135 && headingDeg <= 225) {
        // back faces bottom wall → reset Y (negative side)
        resettingX = false;
        wallSign   = -1.0;
        expected_perpendicular_heading = 0.0;
    }
    else {
        // back faces left wall → reset X (negative side)
        resettingX = true;
        wallSign   = -1.0;
        expected_perpendicular_heading = 90.0;
    }

    // calculate corrected position
    double actualPos = wallSign * (field_half - corrected_distance);

    // calculate corrected heading
    // subtracting the angle correctly converts to global heading
    double corrected_heading = expected_perpendicular_heading - angle_to_wall_deg;

    // normalize heading to 0-360
    corrected_heading = fmod(corrected_heading + 360, 360);

    // apply corrected pose - only update the relevant axis, keep the other
    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, corrected_heading);

    printf("Reset: pos=%.1f hdg=%.1f (wall_angle=%.1f)\n",
           actualPos, corrected_heading, angle_to_wall_deg);
}


void resetPositionLeft(pros::Distance& sensor=leftSensor, double sensor_offset=left_sensor_offset,
                       double field_half=72) {

    double sensorReading = sensor.get() / 25.4;

    if (sensorReading < 0 || sensorReading > 200) {
        printf("Invalid left sensor reading: %.2f\n", sensorReading);
        return;
    }

    lemlib::Pose pose = chassis.getPose();

    // Left sensor direction = robot heading + 270° (pointing left)
    double sensor_heading_deg = pose.theta + 270.0;
    int headingDeg = ((int)sensor_heading_deg % 360 + 360) % 360;

    // Trig correction: find how far off perpendicular we are from the nearest wall
    double nearest_perpendicular = round(sensor_heading_deg / 90.0) * 90.0;
    double angle_off_deg = sensor_heading_deg - nearest_perpendicular;
    double angle_off_rad = angle_off_deg * M_PI / 180.0;

    // Correct the reading for the angle
    double corrected_distance = sensorReading * cos(angle_off_rad) + sensor_offset;

    // Determine which wall
    bool   resettingX = false;
    double wallSign   = 1.0;

    if      (headingDeg >= 315 || headingDeg <= 45)  { resettingX = false; wallSign =  1.0; } // Top wall
    else if (headingDeg > 45  && headingDeg <= 135)  { resettingX = true;  wallSign =  1.0; } // Right wall
    else if (headingDeg > 135 && headingDeg <= 225)  { resettingX = false; wallSign = -1.0; } // Bottom wall
    else                                              { resettingX = true;  wallSign = -1.0; } // Left wall

    double actualPos = wallSign * (field_half - corrected_distance);

    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, pose.theta);
}


void resetPositionRight(pros::Distance& sensor=rightSensor, double sensor_offset=right_sensor_offset,
                        double field_half=72) {

    double sensorReading = sensor.get() / 25.4;

    if (sensorReading < 0 || sensorReading > 200) {
        printf("Invalid right sensor reading: %.2f\n", sensorReading);
        return;
    }

    lemlib::Pose pose = chassis.getPose();

    // Right sensor direction = robot heading + 90°
    double sensor_heading_deg = pose.theta + 90.0;
    int headingDeg = ((int)sensor_heading_deg % 360 + 360) % 360;

    // Trig correction
    double nearest_perpendicular = round(sensor_heading_deg / 90.0) * 90.0;
    double angle_off_deg = sensor_heading_deg - nearest_perpendicular;
    double angle_off_rad = angle_off_deg * M_PI / 180.0;

    double corrected_distance = sensorReading * cos(angle_off_rad) + sensor_offset;

    // Determine which wall
    bool   resettingX = false;
    double wallSign   = 1.0;

    if      (headingDeg >= 315 || headingDeg <= 45)  { resettingX = false; wallSign =  1.0; } // Top wall
    else if (headingDeg > 45  && headingDeg <= 135)  { resettingX = true;  wallSign =  1.0; } // Right wall
    else if (headingDeg > 135 && headingDeg <= 225)  { resettingX = false; wallSign = -1.0; } // Bottom wall
    else                                              { resettingX = true;  wallSign = -1.0; } // Left wall

    double actualPos = wallSign * (field_half - corrected_distance);

    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, pose.theta);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

	intakePiston.set_value(false);
    unloaderPiston.set_value(false);
    descorePiston.set_value(false);
    parkSensor.set_led_pwm(100);

    pros::Task debugTask([](){
    while(true){
        double hue = parkSensor.get_hue();
        double bright = parkSensor.get_brightness();
        double sat = parkSensor.get_saturation();

        double leftIn = leftSensor.get() / 25.4;
        double rightIn = rightSensor.get() / 25.4;
        double backLeftIn = backLeftSensor.get() / 25.4;
        double backRightIn = backRightSensor.get() / 25.4;

        pros::lcd::print(3,"vel: %f\n", intakeMotor1.get_actual_velocity());

        pros::lcd::print(4, "Left: %.2f in", leftIn);
        pros::lcd::print(5, "Right: %.2f in", rightIn);
        pros::lcd::print(6, "BackL: %.2f in", backLeftIn);
        pros::lcd::print(7, "BackR: %.2f in", backRightIn);


        pros::delay(50);
        }
    });

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */



void awp() {
    chassis.setPose(0,0,0);

    intakeMotor1.move(127);
    descorePiston.set_value(true);
	chassis.moveToPoint(0,5,650, {.maxSpeed = 96}, false);
    chassis.moveToPoint(0,-44,1300, {.forwards = false, .maxSpeed = 96}, false);
	chassis.turnToHeading(-95, 600, {.maxSpeed = 127}, true);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(-13,-46.5,1000, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(17,-46.5, 850, {.forwards = false, .maxSpeed = 64}, false);
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    pros::delay(1200);
    intakeMotor2.move(0);
    chassis.swingToHeading(60, lemlib::DriveSide::RIGHT, 650, {.maxSpeed = 96}, false);
    chassis.moveToPoint(24,-18.5,750, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(0, 600, {.maxSpeed = 127}, false);
    chassis.moveToPoint(24,21,1250, {.forwards = true, .maxSpeed = 96}, true);
    pros::delay(1000);
    unloaderPiston.set_value(true);
    intakeMotor2.move(127);
    chassis.moveToPose(39,10,-45, 1050, {.forwards = false, .maxSpeed = 96}, false);
    intakePiston.set_value(true);
    pros::delay(1000);
    unloaderPiston.set_value(false);
    intakePiston.set_value(false);
    intakeMotor2.move(0);
    chassis.moveToPoint(5,46.5,1050, {.forwards = true, .maxSpeed = 96}, false);
    chassis.turnToHeading(-90, 600, {.maxSpeed = 127}, true);
    chassis.moveToPoint(28,46.5,850, {.forwards = false, .maxSpeed = 64}, true);
    pros::delay(700);
    intakeMotor2.move(-127);

}

void middlegoal() {

    chassis.setPose(0,0,0);
    antiJamEnabled = true;

    descorePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
	chassis.moveToPoint(-4,18,900, {.maxSpeed = 96}, false);
    chassis.moveToPoint(-8,25,800, {.maxSpeed = 96}, true);
    unloaderPiston.set_value(true);
    chassis.turnToHeading(-135, 650, {.maxSpeed = 127}, false);
    chassis.moveToPose(6,39,-135,1000, {.forwards = false, .maxSpeed = 64}, true);
    pros::delay(600);
    intakePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(127);
    pros::delay(1300);
    intakePiston.set_value(false);
    intakeMotor2.move(0);
    chassis.moveToPoint(-32,0,1600, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(180, 650, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-32,-12,950, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(200);
    chassis.moveToPoint(-32.5,19,1200, {.forwards = false, .maxSpeed = 64}, false);
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    intakeRunning = true;
    waitUntilIntakeDone(1500);
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    intakeRunning = false;
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 850, {.maxSpeed = 96}, false); //13400
    chassis.moveToPoint(-22.5,16.5,850, {.forwards = true, .maxSpeed = 64}, false); //11150
    chassis.turnToHeading(180, 650, {.maxSpeed = 127}, true); //14050
    descorePiston.set_value(false);
    chassis.moveToPoint(-22.5,35,2000, {.forwards = false, .maxSpeed = 64}, false);

}

void sevenballleft() {
    chassis.setPose(0,0,0);
    antiJamEnabled = true;

    descorePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
	chassis.moveToPoint(-4,18,900, {.maxSpeed = 96}, false);
    chassis.moveToPoint(-8,25,800, {.maxSpeed = 96}, true);
    unloaderPiston.set_value(true);
    chassis.turnToHeading(-135, 650, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-32,0,1500, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(180, 650, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-32,-12,1000, {.forwards = true, .maxSpeed = 56}, false);
    pros::delay(400);
    chassis.moveToPoint(-32,17.5,1000, {.forwards = false, .maxSpeed = 64}, false);
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    intakeRunning = true;
    waitUntilIntakeDone(1500);
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    intakeRunning = false;
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 850, {.maxSpeed = 96}, false); //13400
    chassis.moveToPoint(-22,16.5,850, {.forwards = true, .maxSpeed = 64}, false); //11150
    chassis.turnToHeading(180, 650, {.maxSpeed = 127}, true); //14050
    descorePiston.set_value(false);
    chassis.moveToPoint(-22,35,2000, {.forwards = false, .maxSpeed = 64}, false);

}

void sevenballright() {
    
    chassis.setPose(0,0,0);
    antiJamEnabled = true;

    descorePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
	chassis.moveToPoint(4,18,900, {.maxSpeed = 96}, false);
    chassis.moveToPoint(8,25,800, {.maxSpeed = 96}, true);
    unloaderPiston.set_value(true);
    chassis.turnToHeading(135, 650, {.maxSpeed = 127}, false);
    chassis.moveToPoint(32,0,1500, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(180, 650, {.maxSpeed = 127}, false);
    chassis.moveToPoint(32,-13,1200, {.forwards = true, .maxSpeed = 56}, false);
    chassis.moveToPoint(32,20,1100, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    waitUntilIntakeDone(1500);
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    intakeRunning = false;
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 850, {.maxSpeed = 96}, false); //13400
    chassis.moveToPoint(40.5,18,850, {.forwards = true, .maxSpeed = 64}, false); //11150
    chassis.turnToHeading(180, 650, {.maxSpeed = 127}, true); //14050
    descorePiston.set_value(false);
    chassis.moveToPoint(40.5,35,900, {.forwards = false, .maxSpeed = 64}, false);

    
}


void fourballright() {
    
    chassis.setPose(0,0,0);
    antiJamEnabled = true;

    descorePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
	chassis.moveToPoint(4,18,700, {.maxSpeed = 96}, false);
    chassis.moveToPoint(8,25,500, {.maxSpeed = 96}, true);
    unloaderPiston.set_value(true);
    chassis.turnToHeading(135, 550, {.maxSpeed = 127}, false);
    chassis.moveToPoint(32,0,1500, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(180, 600, {.maxSpeed = 127}, false);
    chassis.moveToPoint(32,20,900, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    waitUntilIntakeDone(1100);
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    intakeRunning = false;
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 850, {.maxSpeed = 96}, false);
    chassis.moveToPoint(41.5,18,650, {.forwards = true, .maxSpeed = 64}, false); 
    chassis.turnToHeading(180, 600, {.maxSpeed = 127}, true); 
    descorePiston.set_value(false);
    chassis.moveToPoint(41.5,35,8000, {.forwards = false, .maxSpeed = 64}, false);

}


void middlegoallast() {

    chassis.setPose(0,0,0);
    antiJamEnabled = true;

    descorePiston.set_value(true);
    chassis.moveToPoint(0,30,1500, {.forwards = true, .maxSpeed = 64}, false);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
    unloaderPiston.set_value(true);
    chassis.turnToHeading(-90, 650, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-12,30, 1150, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(21,31, 1100, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    waitUntilIntakeDone(1100);
    intakeRunning = false;
    chassis.moveToPoint(0,30,1000, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(135, 650, {.maxSpeed = 127}, false);
    chassis.moveToPoint(30,2,1000, {.forwards = true, .maxSpeed = 64}, false);
    intakeMotor2.move(127);
    chassis.turnToHeading(-45, 650, {.maxSpeed = 127}, false);
    chassis.moveToPose(44,-6,-45, 950, {.forwards = false, .maxSpeed = 96}, true);
    pros::delay(750);
    intakePiston.set_value(true);
    pros::delay(1200);
    intakePiston.set_value(false);
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    chassis.moveToPoint(30,5,900, {.forwards = true, .maxSpeed = 96}, false);
    middescorePiston.set_value(true);
    chassis.moveToPose(44,-6,-45, 800, {.forwards = false, .maxSpeed = 96}, true);
    chassis.moveToPoint(10,24,1200, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(-90, 650, {.maxSpeed = 127}, false);
    middescorePiston.set_value(false);
    descorePiston.set_value(false);
    chassis.moveToPoint(35,24,1000, {.forwards = false, .maxSpeed = 64}, false);



}


void sixthreesplit() {

    chassis.setPose(0,0,0);
    antiJamEnabled = true;

    descorePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
	chassis.moveToPoint(-4,18,700, {.maxSpeed = 96}, false);
    chassis.moveToPoint(-8,25,600, {.maxSpeed = 96}, true); //1400
    unloaderPiston.set_value(true);
    chassis.turnToHeading(-45, 600, {.maxSpeed = 127}, true); //2050
    unloaderPiston.set_value(false);
    chassis.moveToPoint(-28,40,800, {.maxSpeed = 96}, true); //2850
    pros::delay(650);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(-8,25,800, {.forwards = false, .maxSpeed = 64}, false); //3750
    chassis.turnToHeading(-135, 600, {.maxSpeed = 127}, true); //4400
    chassis.moveToPose(8.5,39,-135,1000, {.forwards = false, .maxSpeed = 64}, true); //5400
    pros::delay(600);
    intakePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(127);
    pros::delay(1100); //6700
    unloaderPiston.set_value(false);
    intakePiston.set_value(false);
    intakeMotor2.move(64);
    chassis.moveToPoint(-32,0,1600, {.forwards = true, .maxSpeed = 64}, false); //8300
    chassis.turnToHeading(180, 650, {.maxSpeed = 127}, true); //8950
    unloaderPiston.set_value(true);
    chassis.moveToPoint(-32,-11.5,1400, {.forwards = true, .maxSpeed = 56}, false); //10300
    chassis.moveToPoint(-32,19,850, {.forwards = false, .maxSpeed = 64}, false); //11150
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    intakeRunning = true;
    waitUntilIntakeDone(1500); //12650
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    intakeRunning = false;
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 850, {.maxSpeed = 96}, false); //13400
    chassis.moveToPoint(-22,18.5,850, {.forwards = true, .maxSpeed = 64}, false); //11150
    chassis.turnToHeading(180, 650, {.maxSpeed = 127}, true); //14050
    descorePiston.set_value(false);
    chassis.moveToPoint(-22,37,2000, {.forwards = false, .maxSpeed = 64}, false);



    //chassis.moveToPoint(-30.5,0,850, {.forwards = true, .maxSpeed = 96}, false); //13500
    //chassis.turnToHeading(-135, 650, {.maxSpeed = 127}, true); //14150
    //chassis.moveToPoint(-18.75,5,850, {.forwards = false, .maxSpeed = 96}, false); //14900
    //descorePiston.set_value(false);
    //chassis.turnToHeading(180, 650, {.maxSpeed = 127}, true); //
    //chassis.moveToPoint(-18.75,35,2000, {.forwards = false, .maxSpeed = 48}, false);

}


void sigsawp() {

    antiJamEnabled = true;
    double currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,0);

    descorePiston.set_value(true);
    chassis.moveToPoint(0,30,1050, {.forwards = true, .maxSpeed = 64}, false);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
    chassis.turnToHeading(90, 650, {.maxSpeed = 127}, true);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(12,30,1050, {.forwards = true, .maxSpeed = 48}, false);
    chassis.moveToPoint(-25,30, 1000, {.forwards = false, .maxSpeed = 64}, false);
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    chassis.turnToHeading(90, 600, {.maxSpeed = 127}, false);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);
    intakeRunning = true;
    waitUntilIntakeDone(700);
    intakeRunning = false;
    intakeMotor2.move(64);
    chassis.swingToHeading(-150, lemlib::DriveSide::RIGHT, 850, {.maxSpeed = 96}, false);
    chassis.moveToPoint(-6,-20, 800, {.forwards = true, .maxSpeed = 64}, true);
    pros::delay(400);
    unloaderPiston.set_value(true);
    chassis.turnToHeading(-177, 600, {.maxSpeed = 96}, false);
    unloaderPiston.set_value(false);
    chassis.moveToPoint(-6,-68, 1500, {.forwards = true, .maxSpeed = 64}, true);
    pros::delay(1150);
    unloaderPiston.set_value(true);
    intakeMotor2.move(127);
    chassis.moveToPose(-23,-54,135, 950, {.forwards = false, .maxSpeed = 96}, true);
    pros::delay(750);
    intakePiston.set_value(true);
    pros::delay(950);
    intakePiston.set_value(false);
    intakeMotor2.move(64);
    chassis.moveToPoint(15,-92, 1200, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToHeading(90, 600, {.maxSpeed = 127}, false);
    chassis.moveToPoint(37,-92, 1300, {.forwards = true, .maxSpeed = 56}, false);
    chassis.moveToPoint(-20,-92.5, 1100, {.forwards = false, .maxSpeed = 64}, true);
    pros::delay(800);
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);

}


void skills() {

    chassis.setPose(0,0,0);
    antiJamEnabled = true;
    double currentTheta = chassis.getPose().theta;

    descorePiston.set_value(true);
    chassis.moveToPoint(0,30,1800, {.forwards = true, .maxSpeed = 64}, false);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
    chassis.turnToHeading(-90, 650, {.maxSpeed = 127}, false);
    unloaderPiston.set_value(true);
    pros::delay(400);
    chassis.moveToPoint(-12,30,900, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(800);
    chassis.moveToPoint(-5,30,600, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(-12,30,600, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(400);
    currentTheta = -90;
    chassis.setPose(-12,30,currentTheta);
    chassis.moveToPoint(0,30,900, {.forwards = false, .maxSpeed = 64}, false);
    chassis.turnToHeading(180, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(0,60,1500, {.forwards = false, .maxSpeed = 32}, false);

    pros::delay(400);
    currentTheta = 180;
    chassis.setPose(0,0,currentTheta);
    pros::delay(400);
    chassis.moveToPoint(0,-2.5,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(-90, 800, {.maxSpeed = 127}, false);
    chassis.moveToPoint(83,-2.5,3000, {.forwards = false, .maxSpeed = 64}, false);
    unloaderPiston.set_value(false);
    chassis.turnToHeading(180, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(83,-15.5,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(90, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(70,-15.5,800, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    pros::delay(200);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);

    waitUntilIntakeDone();

    intakeRunning = false;
    intakeMotor2.move(64);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(32,0,1200, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(1000);
    chassis.moveToPoint(27,0,600, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(32,0,600, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(-5,0,1200, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    chassis.turnToHeading(90, 700, {.maxSpeed = 127}, false);
    pros::delay(200);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);

    waitUntilIntakeDone();

    intakeRunning = false;
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    unloaderPiston.set_value(false);

    chassis.moveToPoint(10,0,900, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(0, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(10,-95,3500, {.forwards = false, .maxSpeed = 64}, false);
    chassis.turnToHeading(90, 700, {.maxSpeed = 127}, false);
    unloaderPiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
    pros::delay(400);
    chassis.moveToPoint(32,-95,1200, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(800);
    chassis.moveToPoint(27,-95,600, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(32,-95,600, {.forwards = true, .maxSpeed = 64}, false);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(32,-95,currentTheta);
    chassis.moveToPoint(16,-94,900, {.forwards = false, .maxSpeed = 64}, false);
    chassis.turnToHeading(0, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(16,-130,1500, {.forwards = false, .maxSpeed = 32}, false);

    pros::delay(400);
    currentTheta = 0;
    chassis.setPose(0,0,currentTheta);
    pros::delay(400);
    chassis.moveToPoint(0,3,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(90, 800, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-83,3,3000, {.forwards = false, .maxSpeed = 64}, false);
    unloaderPiston.set_value(false);
    chassis.turnToHeading(0, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-83,16,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(-90, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-60,16,800, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    chassis.turnToHeading(-90, 700, {.maxSpeed = 127}, false);
    pros::delay(200);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);

    waitUntilIntakeDone();

    intakeRunning = false;
    intakeMotor2.move(64);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(-32,0,1300, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(800);
    chassis.moveToPoint(-27,0,600, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(-32,0,600, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(5,0,1200, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    pros::delay(200);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);

    waitUntilIntakeDone();

    intakeRunning = false;
    intakeMotor2.move(64);

    chassis.moveToPoint(-20,0,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(-20, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-30,20,900, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(-35,40,900, {.forwards = true, .maxSpeed = 64}, false);
    enableParkDetect = true;
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);
    chassis.moveToPoint(0,100,9000, {.forwards = true, .maxSpeed = 64}, true);
    
}

void riskskills() {

    chassis.setPose(0,0,0);
    antiJamEnabled = true;
    double currentTheta = chassis.getPose().theta;

    descorePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
	chassis.moveToPoint(-4,18,700, {.maxSpeed = 96}, false);
    chassis.moveToPoint(-8,25,600, {.maxSpeed = 96}, false);
    chassis.moveToPose(6,39,-135,1000, {.forwards = false, .maxSpeed = 64}, false);
    intakePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(112);
    pros::delay(1000);
    intakeMotor2.move(-48);
    intakePiston.set_value(false);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(-30,0,1400, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(180, 600, {.maxSpeed = 127}, false);
    intakeMotor2.move(64);
    chassis.moveToPoint(-30,-12,1200, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(800);
    chassis.moveToPoint(-30,0,800, {.forwards = false, .maxSpeed = 96}, false);
    chassis.turnToHeading(90, 550, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-60,0,900, {.forwards = false, .maxSpeed = 32}, false);

    pros::delay(400);
    currentTheta = 180;
    chassis.setPose(0,0,currentTheta);
    pros::delay(400);
    chassis.moveToPoint(0,-2.5,600, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(-90, 600, {.maxSpeed = 127}, false);
    chassis.moveToPoint(83,-2.5,3000, {.forwards = false, .maxSpeed = 64}, false);
    unloaderPiston.set_value(false);
    chassis.turnToHeading(180, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(83,-15.5,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(90, 600, {.maxSpeed = 127}, false);
    chassis.moveToPoint(70,-15.5,750, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    pros::delay(200);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);

    waitUntilIntakeDone();

    intakeRunning = false;
    intakeMotor2.move(64);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(32,0,1200, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(1000);
    chassis.moveToPoint(27,0,600, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(32,0,600, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(-5,0,1000, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    chassis.turnToHeading(90, 600, {.maxSpeed = 127}, false);
    pros::delay(200);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);

    waitUntilIntakeDone();

    intakeRunning = false;
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    unloaderPiston.set_value(false);

    chassis.moveToPoint(25,0,900, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(160, 600, {.maxSpeed = 127}, false);
    chassis.moveToPoint(34,-25,900, {.forwards = true, .maxSpeed = 64}, false);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
    chassis.moveToPoint(34,-90,2500, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(34,0,1500, {.forwards = false, .maxSpeed = 32}, false);
    pros::delay(200);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,3,900, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(90, 600, {.maxSpeed = 127}, false);
    chassis.moveToPoint(30,3,900, {.forwards = true, .maxSpeed = 64}, false);
    intakeMotor1.move(32);
    chassis.turnToHeading(-45, 600, {.maxSpeed = 127}, false);
    chassis.moveToPose(47.75,-4.75,-45,1200, {.forwards = false, .maxSpeed = 64}, false);
    intakePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(112);
    pros::delay(3500);
    intakePiston.set_value(false);
    chassis.moveToPoint(10,30,1500, {.forwards = true, .maxSpeed = 64}, false);
    intakeMotor2.move(64);
    unloaderPiston.set_value(true);
    chassis.turnToHeading(-90, 600, {.maxSpeed = 127}, false);
    chassis.moveToPoint(0,30,1200, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(1000);
    chassis.moveToPoint(5,30,700, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(0,30,700, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(15,30,800, {.forwards = false, .maxSpeed = 64}, false);
    chassis.turnToHeading(180, 550, {.maxSpeed = 127}, false);
    chassis.moveToPoint(15,60,1000, {.forwards = false, .maxSpeed = 32}, false);

    pros::delay(400);
    currentTheta = 180;
    chassis.setPose(0,0,currentTheta);
    pros::delay(400);
    chassis.moveToPoint(0,-2.5,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(-90, 800, {.maxSpeed = 127}, false);
    chassis.moveToPoint(83,-2.5,2900, {.forwards = false, .maxSpeed = 64}, false);
    unloaderPiston.set_value(false);
    chassis.turnToHeading(180, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(83,-15.5,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(90, 700, {.maxSpeed = 127}, false);
    chassis.moveToPoint(70,-15.5,800, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    pros::delay(200);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);

    waitUntilIntakeDone();

    intakeRunning = false;
    intakeMotor2.move(64);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(32,0,1200, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(1000);
    chassis.moveToPoint(27,0,600, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(32,0,600, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(-5,0,1000, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);
    unloaderPiston.set_value(false);
    chassis.turnToHeading(90, 600, {.maxSpeed = 127}, false);
    pros::delay(200);
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);

    waitUntilIntakeDone();

    intakeRunning = false;
    intakeMotor1.move(127);
    intakeMotor2.move(0);
    unloaderPiston.set_value(false);

    chassis.moveToPoint(20,0,500, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(160, 500, {.maxSpeed = 127}, false);
    chassis.moveToPoint(30,-20,500, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(35,-40,500, {.forwards = true, .maxSpeed = 64}, false);
    enableParkDetect = true;
    currentTheta = chassis.getPose().theta;
    chassis.setPose(0,0,currentTheta);
    chassis.moveToPoint(0,-100,9000, {.forwards = true, .maxSpeed = 64}, true);
    
}

void finalskills() {

    chassis.setPose(-16,-48,0);
    antiJamEnabled = true;

    descorePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(64);
	chassis.moveToPoint(-20,-30,700, {.maxSpeed = 96}, false);
    chassis.moveToPoint(-24,-23,600, {.maxSpeed = 96}, false);
    chassis.moveToPose(-10,-9,-135,1000, {.forwards = false, .maxSpeed = 64}, false);
    intakePiston.set_value(true);
    intakeMotor1.move(127);
    intakeMotor2.move(112);
    pros::delay(1000);
    intakePiston.set_value(false);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(-46,-48,1600, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(180, 600, {.maxSpeed = 127}, false);
    intakeMotor2.move(64);

    //unloader 1
    chassis.moveToPoint(-46,-60,1200, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(-46,-58,800, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(-46,-60,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(-46,-58,800, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(-46,-60,800, {.forwards = true, .maxSpeed = 64}, false);

    chassis.moveToPoint(-46,-48,800, {.forwards = false, .maxSpeed = 96}, false);
    chassis.turnToHeading(90, 550, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-60,-48,900, {.forwards = false, .maxSpeed = 64}, false);
    resetPositionAndHeadingBack();
    resetPositionRight();
    chassis.turnToHeading(180, 550, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-60,35,3000, {.forwards = false, .maxSpeed = 64}, false);
    chassis.turnToHeading(90, 550, {.maxSpeed = 127}, false);
    resetPositionAndHeadingBack();
    resetPositionRight();
    chassis.moveToPoint(-46,35,800, {.forwards = true, .maxSpeed = 64}, false);
    chassis.turnToHeading(0, 550, {.maxSpeed = 127}, false);
    chassis.moveToPoint(-46,22,800, {.forwards = false, .maxSpeed = 64}, false);
    intakeRunning = true;
    intakeMotor2.move(-127);

    waitUntilIntakeDone();

    intakeRunning = false;
    resetPositionLeft();
    intakeMotor2.move(64);
    unloaderPiston.set_value(true);
    chassis.moveToPoint(-46,54,1200, {.forwards = true, .maxSpeed = 64}, false);
    pros::delay(1000);
    chassis.moveToPoint(-46,52,600, {.forwards = false, .maxSpeed = 64}, false);
    chassis.moveToPoint(-46,54,600, {.forwards = true, .maxSpeed = 64}, false);
    chassis.moveToPoint(-46,22,1000, {.forwards = false, .maxSpeed = 64}, false);


}


void testing() {

    chassis.setPose(60,-59,0);

	chassis.moveToPoint(60,-39,1500, {.maxSpeed = 64}, false);

    pros::delay(2000);

    pros::delay(2000);

    chassis.moveToPoint(60,-29,1500, {.maxSpeed = 64}, false);

    pros::delay(2000);

}

// ==========================================
// TEST 1: STATIC SENSORS (No Movement)
// ==========================================
void staticSensorTest() {
    while(true) {
        pros::lcd::clear();
        pros::lcd::print(0, "--- STATIC SENSOR TEST ---");
        pros::lcd::print(2, "L: %.2f  |  R: %.2f", (double)leftSensor.get()/25.4, (double)rightSensor.get()/25.4);
        pros::lcd::print(3, "BL: %.2f |  BR: %.2f", (double)backLeftSensor.get()/25.4, (double)backRightSensor.get()/25.4);
        pros::lcd::print(5, "Hue: %.1f | Bright: %.1f", (double)parkSensor.get_hue(), (double)parkSensor.get_brightness());
        pros::lcd::print(7, "STATUS: Live Monitoring...");
        pros::delay(100); // 10Hz refresh for readability
    }
}

// ==========================================
// TEST 2: RESET VALIDATION (With Movement)
// ==========================================
void resetValidationTest() {
    chassis.setPose(0, 0, 0);
    pros::lcd::print(0, "TEST: Moving to Wall...");
    
    // Drive backwards to a spot where we know the back is facing a wall
    chassis.moveToPoint(0, -36, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(500); // Wait for robot to settle

    // 1. Record what the robot "Thought" its position was
    lemlib::Pose oldPose = chassis.getPose();

    // 2. Perform the sensor resets
    resetPositionAndHeadingBack();
    resetPositionLeft();

    // 3. Record the "New" corrected position
    lemlib::Pose newPose = chassis.getPose();

    // 4. Calculate the jump (drift)
    double jumpX = newPose.x - oldPose.x;
    double jumpY = newPose.y - oldPose.y;
    double jumpH = newPose.theta - oldPose.theta;

    // 5. Display the Accuracy Report
    while(true) {
        pros::lcd::clear();
        pros::lcd::print(0, "=== RESET ACCURACY REPORT ===");
        pros::lcd::print(2, "X JUMPED: %.2f in", jumpX);
        pros::lcd::print(3, "Y JUMPED: %.2f in", jumpY);
        pros::lcd::print(4, "HDG JUMPED: %.2f deg", jumpH);
        
        pros::lcd::print(6, "NEW POSE: X:%.1f Y:%.1f H:%.1f", newPose.x, newPose.y, newPose.theta);
        pros::lcd::print(7, "STATUS: Check Resets.");
        pros::delay(1000);
    }
}


void autonomous() {
    int autonSelection = 11; // <--- CHANGE THIS NUMBER TO SELECT AUTON

    switch (autonSelection) {
        case 1:  awp(); break;
        case 2:  middlegoal(); break;
        case 3:  sevenballleft(); break;
        case 4:  sevenballright(); break;
        case 5:  fourballright(); break;
        case 6:  middlegoallast(); break;
        case 7:  sixthreesplit(); break;
        case 8:  sigsawp(); break;
        case 9:  skills(); break;
        case 10: riskskills(); break;
        case 11: finalskills(); break;
        case 12: testing(); break;
        case 13: staticSensorTest(); break;
        case 14: resetValidationTest(); break;
        
        default: 
            pros::lcd::print(0, "ERROR: No Auton Selected");
            break;
    }
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.tank(leftY, leftX);

        /* ================= R1 – Intake Forward Toggle ================= */
        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        if (r1 && !r1Last) {
            intakeOn = !intakeOn;
            intakeReverseOn = false;
            bottomOnlyOn = false;
            pistonOn = false;
        }
        r1Last = r1;

        /* ================= R2 – Intake Reverse Toggle ================= */
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        if (r2 && !r2Last) {
            intakeReverseOn = !intakeReverseOn;
            intakeOn = false;
            bottomOnlyOn = false;
            pistonOn = false;
        }
        r2Last = r2;

        /* ================= L1 – Bottom Intake Only Toggle ================= */
        bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        if (l1 && !l1Last) {
            bottomOnlyOn = !bottomOnlyOn;
            intakeOn = false;
            intakeReverseOn = false;
            pistonOn = false;
        }
        l1Last = l1;

        /* ================= Intake Motor Logic ================= */
        if (intakeOn) {
            intakeMotor1.move(127);
            intakeMotor2.move(-127);
        }
        else if (intakeReverseOn) {
            intakeMotor1.move(-127);
            intakeMotor2.move(127);
        }
        else if (bottomOnlyOn) {
            intakeMotor1.move(127);
            intakeMotor2.move(64);
        }
        else if (pistonOn) {
            intakeMotor1.move(127);
            intakeMotor2.move(127);
        }
        else {
            intakeMotor1.move(0);
            intakeMotor2.move(0);
        }

        /* ================= Button A – Intake Piston + Intake Motion ================= */
        bool a = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (a && !aLast) {
            intakePistonState = !intakePistonState;
            intakePiston.set_value(intakePistonState);

            if (intakePistonState) {
                pistonOn = !pistonOn;
                intakeOn = false;
                intakeReverseOn = false;
                bottomOnlyOn = false;
            }
            else {
                intakeOn = false;
                intakeReverseOn = false;
                bottomOnlyOn = false;
                pistonOn = false;
            }
        }
        aLast = a;

        /* ================= UP Arrow – Piston C Toggle ================= */
        bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        descorePiston.set_value(!up);

        /* ================= X – Piston B Toggle ================= */
        bool x = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        if (x && !xLast) {
            pistonBState = !pistonBState;
            unloaderPiston.set_value(pistonBState);
        }
        xLast = x;

        /* ================= Right – Piston G Toggle ================= */
        bool right = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        if (right && !rightLast) {
            pistonFstate = !pistonFstate;
            middescorePiston.set_value(pistonFstate);
        }
        rightLast = right;

        // delay to save resources
        pros::delay(10);
    }
}