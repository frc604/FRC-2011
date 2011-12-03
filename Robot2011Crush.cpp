#include "WPILib.h"
#include <math.h>

/* Port configuration for sensors and actuators. */
	#define LEFT_DRIVE_JOYSTICK_USB_PORT 3
	#define MANIPULATOR_JOYSTICK_USB_PORT 1
	#define RIGHT_DRIVE_JOYSTICK_USB_PORT 2

	#define FRONT_LEFT_MOTOR_PORT 3
	#define FRONT_RIGHT_MOTOR_PORT 2
	#define REAR_LEFT_MOTOR_PORT 4
	#define REAR_RIGHT_MOTOR_PORT 1

	#define ARM_LIFT_MOTOR_1_PORT 5
	#define ARM_LIFT_MOTOR_2_PORT 6
	#define ARM_EXTEND_MOTOR_PORT 7
	#define ARM_UPPER_ROLLER_MOTOR_PORT 8
	#define ARM_LOWER_ROLLER_MOTOR_PORT 9

	#define POTENTIOMETER_PORT 7
	#define GYRO_PORT 1

	#define COMPRESSOR_PORT 3
	#define PRESSURE_SWITCH_PORT 4
	#define GRIPPER_SOLENOID_FORWARD_PORT 6
	#define GRIPPER_SOLENOID_REVERSE_PORT 5
	#define MINIBOT_DROP_SOLENOID_PORT 3
	#define MINIBOT_ACTIVATE_SOLENOID_PORT 8
	#define SHIFTER_SOLENOID_FORWARD_PORT 1
	#define SHIFTER_SOLENOID_REVERSE_PORT 2

/* Button configuration. */
	/* Manipulator Button Configuration */
		#define MANIPULATOR_SUCK_IN_BUTTON 1
		#define MANIPULATOR_SPIT_OUT_BUTTON 3
		#define MANIPULATOR_ROTATE_UP_BUTTON 4
		#define MANIPULATOR_ROTATE_DOWN_BUTTON 2
		#define MANIPULATOR_EXTEND_BUTTON 6
		#define MANIPULATOR_RETRACT_BUTTON 7

		#define MANIPULATOR_OPEN_GRIPPER_BUTTON 5
		#define MANIPULATOR_MINIBOT_DROP_BUTTON 8

	/* Driver Button Configuration */
		#define DRIVER_SHIFT_BUTTON 1
		#define DRIVER_MINIBOT_ACTIVATE_BUTTON 4

		#define DRIVER_GYRO_RESET_BUTTON 5
		#define DRIVER_GYRO_FORWARD_BUTTON 3
		#define DRIVER_GYRO_REVERSE_BUTTON 2

/* Potentiometer Calibration */
	#define POTENTIOMETER_VOLTAGE_FULLY_FORWARD 1.56
	#define POTENTIOMETER_VOLTAGE_FULLY_BACKWARD 4.01

/* Actuator polarity and speed configuration. */
	#define GYRO_DRIVE_POWER 0.9

	#define ARM_EXTEND_SPEED 0.8
	#define ARM_EXTEND_SPEED_AUTONOMOUS 1.0
	#define ARM_RETRACT_SPEED -0.5
	#define ARM_RETRACT_SPEED_AUTONOMOUS -1.0
	#define ARM_ROLLER_UPPER_IN_SPEED -0.8
	#define ARM_ROLLER_LOWER_IN_SPEED 0.8
	#define ARM_ROLLER_UPPER_OUT_SPEED 0.8
	#define ARM_ROLLER_LOWER_OUT_SPEED -0.8

	#define SOLENOID_SHIFTER_HIGH_POWER_DIRECTION DoubleSolenoid::kForward
	#define SOLENOID_SHIFTER_LOW_POWER_DIRECTION DoubleSolenoid::kReverse

	#define SOLENOID_MINIBOT_DROP_ON_DIRECTION true
	#define SOLENOID_MINIBOT_DROP_OFF_DIRECTION false

	#define SOLENOID_MINIBOT_ACTIVATE_ON_DIRECTION true
	#define SOLENOID_MINIBOT_ACTIVATE_OFF_DIRECTION false

	#define SOLENOID_GRIPPER_OPEN_DIRECTION DoubleSolenoid::kReverse
	#define SOLENOID_GRIPPER_CLOSED_DIRECTION DoubleSolenoid::kForward

class PIDSourceArm : public PIDSource {
	AnalogChannel *analogPotentiometer;
	
	public:
		PIDSourceArm(AnalogChannel *aP) {
			analogPotentiometer = aP;
		}
		
		float BindToRange(float xValue, float upperBound = 1.0, float lowerBound = -1.0) { // Simple limiting function.
			return (xValue>upperBound) ? (upperBound) : ((xValue<lowerBound) ? (lowerBound) : (xValue));
		}

		float ProcessArmPosition(float potentiometerVoltage) { // Convert a raw potentiometer voltage to an arm position value.
			return BindToRange((((potentiometerVoltage-POTENTIOMETER_VOLTAGE_FULLY_BACKWARD)/(POTENTIOMETER_VOLTAGE_FULLY_FORWARD-POTENTIOMETER_VOLTAGE_FULLY_BACKWARD))-0.5)*2.0);
		}

		virtual double PIDGet() {
			return ProcessArmPosition(analogPotentiometer->GetVoltage());
		}
};

class PIDOutputArm : public PIDOutput {
	Victor *motorArmLift1;
	Victor *motorArmLift2;
	DriverStationLCD *dsLCD;
	
	public:
		PIDOutputArm(Victor *mAL1, Victor *mAL2, DriverStationLCD *dsL) {
			motorArmLift1 = mAL1;
			motorArmLift2 = mAL2;
			dsLCD = dsL;
		}
		
		virtual void PIDWrite(float output) {
			output *= -0.8;
			motorArmLift1->Set(output);
			motorArmLift2->Set(output);
			dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Output: %f", output);
			dsLCD->UpdateLCD();
		}
};

class Robot2011Crush : public SimpleRobot {
	Joystick joystickManipulator;
	Joystick joystickDriveLeft;
	Joystick joystickDriveRight;

	RobotDrive driveTrain;

	Victor motorArmLift1;
	Victor motorArmLift2;
	Victor motorArmExtend;
	Victor motorArmRollerUpper;
	Victor motorArmRollerLower;

	AnalogChannel analogPotentiometer;
	Gyro gyroDriving;

	Compressor* compressorPump;

	DoubleSolenoid solenoidGripper;
	Solenoid solenoidMinibotDrop;
	Solenoid solenoidMinibotActivate;
	DoubleSolenoid solenoidShifter;

	public:
		Robot2011Crush():
			joystickManipulator(MANIPULATOR_JOYSTICK_USB_PORT),
			joystickDriveLeft(LEFT_DRIVE_JOYSTICK_USB_PORT),
			joystickDriveRight(RIGHT_DRIVE_JOYSTICK_USB_PORT),
			driveTrain(new Victor(FRONT_LEFT_MOTOR_PORT), new Victor(REAR_LEFT_MOTOR_PORT), new Victor(FRONT_RIGHT_MOTOR_PORT), new Victor(REAR_RIGHT_MOTOR_PORT)),
			motorArmLift1(ARM_LIFT_MOTOR_1_PORT),
			motorArmLift2(ARM_LIFT_MOTOR_2_PORT),
			motorArmExtend(ARM_EXTEND_MOTOR_PORT),
			motorArmRollerUpper(ARM_UPPER_ROLLER_MOTOR_PORT),
			motorArmRollerLower(ARM_LOWER_ROLLER_MOTOR_PORT),
			analogPotentiometer(POTENTIOMETER_PORT),
			gyroDriving(GYRO_PORT),
			solenoidGripper(GRIPPER_SOLENOID_FORWARD_PORT, GRIPPER_SOLENOID_REVERSE_PORT),
			solenoidMinibotDrop(MINIBOT_DROP_SOLENOID_PORT),
			solenoidMinibotActivate(MINIBOT_ACTIVATE_SOLENOID_PORT),
			solenoidShifter(SHIFTER_SOLENOID_FORWARD_PORT, SHIFTER_SOLENOID_REVERSE_PORT)
		{
			GetWatchdog().SetEnabled(false); // If you're just beginning, and nothing's going on, there's no need for Watchdog to be doing anything.

			driveTrain.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
			driveTrain.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
			driveTrain.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
			driveTrain.SetInvertedMotor(RobotDrive::kRearRightMotor, true);

			compressorPump = new Compressor(PRESSURE_SWITCH_PORT, COMPRESSOR_PORT);
		}

		bool IsInRange(float xValue, float upperRange, float lowerRange) { // Self-explanatory.
			return xValue<=upperRange && xValue >= lowerRange;
		}
		
		float Deadband(float xValue, float upperBand = 0.1, float lowerBand = 0.1, float correctedValue = 0.0) { // The antithesis of the BindToRange function
			return (IsInRange(xValue, upperBand, lowerBand)) ? (correctedValue) : (xValue);
		}
		
		void Autonomous() {
			GetWatchdog().SetEnabled(false); // No need for Watchdog in Autonomous, either.
			compressorPump->Start(); // Let's start up the compressor and charge up for Teleop.

			bool boolSnappedArm = false; // Set that.
			bool boolWentThere = false; // And that.

			/* PID Control */
				PIDSourceArm *pidSourceArm = new PIDSourceArm(&analogPotentiometer);
				PIDController *pidArmController = new PIDController(1.3, 0.0, 1.6, pidSourceArm, new PIDOutputArm(&motorArmLift1, &motorArmLift2, DriverStationLCD::GetInstance()));

				pidArmController->SetSetpoint(0.75);

			/* Start up the timer. */
				Timer* timerSnapArm = new Timer();

				timerSnapArm->Reset();
				timerSnapArm->Start();
				
			while(IsAutonomous() && IsEnabled()) {
				/* Snap the arm into position. */
					if(!boolSnappedArm) {
						if(timerSnapArm->Get() >= 0.6) boolSnappedArm = true; else motorArmExtend.Set(ARM_EXTEND_SPEED_AUTONOMOUS);
					} else {
						if(!boolWentThere) {
							boolWentThere = true;
							
							timerSnapArm->Reset();
							
							motorArmExtend.Set(ARM_RETRACT_SPEED_AUTONOMOUS);
							pidArmController->Enable();
						} else {
							if(timerSnapArm->Get() >= 0.3) motorArmExtend.Set(0);
							
							if(timerSnapArm->Get() >= 0.6) {
								timerSnapArm->Stop();
								pidArmController->Disable();
							}
						}
					}

				/* If nothing's left to do, sit around and keep the compressor running. */
			}
			
			compressorPump->Stop(); // Okay, fun's over
		}

		void OperatorControl() {
			GetWatchdog().SetEnabled(true); // We do want Watchdog in Teleop, though.
			compressorPump->Start(); // Let's start up the compressor too, while we're at it.

			/* Declare and initialize variables. */
				double doubleGyroPosition;
				double doubleCurrentPosition;
				double doubleLastPosition;
				
				double doubleArmLiftPower;
				double doubleRollerMultiplier;
				
				bool boolGyroForwardButton = false;
				bool boolGyroReverseButton = false;

				bool boolHeldInPlace = false;
				int stateActivePreset = 0;

				bool boolTopPreset;
				bool boolBottomPreset;
				
				bool boolExtendButton;
				bool boolRetractButton;

				bool boolRotateUpButton;
				bool boolRotateDownButton;
				
				bool boolMinibotTimer = false;
				
				bool boolMinibotDropped = false;
				
				Timer* timerDriveTimer = new Timer();
				Timer* timerMinibotDrop = new Timer();

			/* Debug Functionality */
				DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                 ");
				dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                 ");
				dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                 ");
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                 ");
				dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                 ");
				dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                 ");
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "---Potentiometer---", analogPotentiometer.GetVoltage());
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "---PID Control---", analogPotentiometer.GetVoltage());
				dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Status: Disabled");
				dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Output: 0.000000");
			
			/* PID Control */
				PIDSourceArm *pidSourceArm = new PIDSourceArm(&analogPotentiometer);
				PIDController *pidArmController = new PIDController(1.3, 0.0, 1.6, pidSourceArm, new PIDOutputArm(&motorArmLift1, &motorArmLift2, dsLCD));

			/* Drive Timer */
				timerDriveTimer->Start();
				
			while(IsOperatorControl() && IsEnabled()) {
				GetWatchdog().Feed(); // Feed the Watchdog.
			
				/* Drive Control */
					/* Shifting */
						if(joystickDriveLeft.GetRawButton(DRIVER_SHIFT_BUTTON) || joystickDriveRight.GetRawButton(DRIVER_SHIFT_BUTTON)) solenoidShifter.Set(SOLENOID_SHIFTER_HIGH_POWER_DIRECTION); else solenoidShifter.Set(SOLENOID_SHIFTER_LOW_POWER_DIRECTION);

					/* Gyro Control */
						if(joystickDriveLeft.GetRawButton(DRIVER_GYRO_RESET_BUTTON) && joystickDriveRight.GetRawButton(DRIVER_GYRO_RESET_BUTTON)) {
							gyroDriving.Reset(); // Reset the gyro.
						} else {
							boolGyroForwardButton = joystickDriveLeft.GetRawButton(DRIVER_GYRO_FORWARD_BUTTON) && joystickDriveRight.GetRawButton(DRIVER_GYRO_FORWARD_BUTTON);
							boolGyroReverseButton = joystickDriveLeft.GetRawButton(DRIVER_GYRO_REVERSE_BUTTON) && joystickDriveRight.GetRawButton(DRIVER_GYRO_REVERSE_BUTTON);
							
							doubleCurrentPosition = gyroDriving.GetAngle();
							doubleGyroPosition += doubleCurrentPosition-doubleLastPosition;
							doubleLastPosition = doubleCurrentPosition;
							
							doubleGyroPosition -= timerDriveTimer->Get()*0.0238095238; // Account for drift.
							timerDriveTimer->Reset();
							
							if(doubleGyroPosition >= 360) doubleGyroPosition -= 360;
							if(doubleGyroPosition < 0) doubleGyroPosition += 360;
							
							if((boolGyroForwardButton || boolGyroReverseButton) && !(boolGyroForwardButton && boolGyroReverseButton)) {
								if(boolGyroForwardButton) {
									if(doubleGyroPosition > 180 && doubleGyroPosition < 358) {
										driveTrain.TankDrive(-GYRO_DRIVE_POWER, GYRO_DRIVE_POWER);
									} else if(doubleGyroPosition > 2 && doubleGyroPosition <= 180) {
										driveTrain.TankDrive(GYRO_DRIVE_POWER, -GYRO_DRIVE_POWER);
									} else {
										driveTrain.TankDrive(0.0, 0.0);
									}
								}
								
								if(boolGyroReverseButton) {
									if(doubleGyroPosition < 178) {
										driveTrain.TankDrive(-GYRO_DRIVE_POWER, GYRO_DRIVE_POWER);
									} else if(doubleGyroPosition > 182) {
										driveTrain.TankDrive(GYRO_DRIVE_POWER, -GYRO_DRIVE_POWER);
									} else {
										driveTrain.TankDrive(0.0, 0.0);
									}
								}
							} else {
								/* Drive Train */
									driveTrain.TankDrive(joystickDriveLeft, joystickDriveRight);
							}
						}
						
				/* Arm Control */
					/* Rotational Control */
						if(joystickManipulator.GetRawButton(9)) { // Hold in place.
							if(!boolHeldInPlace) {
								boolHeldInPlace = true;
								pidArmController->SetSetpoint(pidSourceArm->PIDGet());
								pidArmController->Enable();
								dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                 ");
								dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Status: Holding");
							}
						} else {
							boolHeldInPlace = false;
							
							boolTopPreset = joystickManipulator.GetRawButton(11);
							boolBottomPreset = joystickManipulator.GetRawButton(10);
							
							if((boolTopPreset || boolBottomPreset) && !(boolTopPreset && boolBottomPreset)) { // Is a preset button pressed?
								if(boolTopPreset && stateActivePreset!=1) { // Go to the ground preset.
									stateActivePreset = 1;
									pidArmController->SetSetpoint(0.80);
									pidArmController->Enable();
									dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                 ");
									dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Status: Ground");
								}
								
								if(boolBottomPreset && stateActivePreset!=2) { // Go to the hanging preset.
									stateActivePreset = 2;
									pidArmController->Enable();
									pidArmController->SetSetpoint(-0.78);
									dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                 ");
									dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Status: Hanging");
								}
							} else { // Manual arm control.
								if(boolHeldInPlace || stateActivePreset != 0) { // Turn off PID control.
									pidArmController->Disable();
									dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                 ");
									dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Status: Disabled");
								}
								
								stateActivePreset = 0;
								
								doubleArmLiftPower = joystickManipulator.GetY();
								doubleArmLiftPower = (doubleArmLiftPower<0) ? pow(doubleArmLiftPower, 2)*-1 : pow(doubleArmLiftPower, 2);
								
								motorArmLift1.Set(doubleArmLiftPower);
								motorArmLift2.Set(doubleArmLiftPower);
							}
						}

					/* Extension Control */
						boolExtendButton = joystickManipulator.GetRawButton(MANIPULATOR_EXTEND_BUTTON);
						boolRetractButton = joystickManipulator.GetRawButton(MANIPULATOR_RETRACT_BUTTON);

						if(boolExtendButton && !boolRetractButton) {
							motorArmExtend.Set(ARM_EXTEND_SPEED);
						} else if(!boolExtendButton && boolRetractButton) {
							motorArmExtend.Set(ARM_RETRACT_SPEED);
						} else {
							motorArmExtend.Set(0);
						}

					/* Roller Control */
						boolRotateUpButton = joystickManipulator.GetRawButton(MANIPULATOR_ROTATE_UP_BUTTON);
						boolRotateDownButton = joystickManipulator.GetRawButton(MANIPULATOR_ROTATE_DOWN_BUTTON);
						
						doubleRollerMultiplier = (joystickManipulator.GetZ()+1.0)/2.0;

						if(joystickManipulator.GetRawButton(MANIPULATOR_SUCK_IN_BUTTON)) {
							motorArmRollerUpper.Set(ARM_ROLLER_UPPER_IN_SPEED*doubleRollerMultiplier);
							motorArmRollerLower.Set(ARM_ROLLER_LOWER_IN_SPEED*doubleRollerMultiplier);
						} else if(joystickManipulator.GetRawButton(MANIPULATOR_SPIT_OUT_BUTTON)) {
							motorArmRollerUpper.Set(ARM_ROLLER_UPPER_OUT_SPEED*doubleRollerMultiplier);
							motorArmRollerLower.Set(ARM_ROLLER_LOWER_OUT_SPEED*doubleRollerMultiplier);
						} else if(boolRotateUpButton && !boolRotateDownButton) {
							motorArmRollerUpper.Set(ARM_ROLLER_UPPER_IN_SPEED*doubleRollerMultiplier);
							motorArmRollerLower.Set(ARM_ROLLER_LOWER_OUT_SPEED*doubleRollerMultiplier);
						} else if(!boolRotateUpButton && boolRotateDownButton) {
							motorArmRollerUpper.Set(ARM_ROLLER_UPPER_OUT_SPEED*doubleRollerMultiplier);
							motorArmRollerLower.Set(ARM_ROLLER_LOWER_IN_SPEED*doubleRollerMultiplier);
						} else {
							motorArmRollerUpper.Set(0);
							motorArmRollerLower.Set(0);
						}

					/* Gripper Control */
						if(joystickManipulator.GetRawButton(MANIPULATOR_OPEN_GRIPPER_BUTTON)) solenoidGripper.Set(SOLENOID_GRIPPER_OPEN_DIRECTION); else solenoidGripper.Set(SOLENOID_GRIPPER_CLOSED_DIRECTION);

				/* Minibot Deployment */
					/* Minibot Dropper */
						if(joystickManipulator.GetRawButton(MANIPULATOR_MINIBOT_DROP_BUTTON)) {
							if(!boolMinibotTimer) {
								boolMinibotTimer = true;
								timerMinibotDrop->Reset();
								timerMinibotDrop->Start();
							}

							if(timerMinibotDrop->Get()>=0.2) {
								timerMinibotDrop->Stop();
								solenoidMinibotDrop.Set(SOLENOID_MINIBOT_DROP_ON_DIRECTION);
								boolMinibotTimer = false;
								boolMinibotDropped = true;
							}
						} else {
							timerMinibotDrop->Stop();
							solenoidMinibotDrop.Set(SOLENOID_MINIBOT_DROP_OFF_DIRECTION);
						}

					/* Minibot Activation */
						if(boolMinibotDropped && joystickDriveLeft.GetRawButton(DRIVER_MINIBOT_ACTIVATE_BUTTON) && joystickDriveRight.GetRawButton(DRIVER_MINIBOT_ACTIVATE_BUTTON)) solenoidMinibotActivate.Set(SOLENOID_MINIBOT_ACTIVATE_ON_DIRECTION); else solenoidMinibotActivate.Set(SOLENOID_MINIBOT_ACTIVATE_OFF_DIRECTION);
						
					/* Debug Output for Potentiometer Calibration */
						dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Voltage: %f", analogPotentiometer.GetVoltage());
						dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Position: %f", pidSourceArm->PIDGet());
						dsLCD->UpdateLCD();
			}

			pidArmController->Disable(); // Turn off the PID control, otherwise it'll keep running.
			compressorPump->Stop(); // We're disabling now, so let's switch off the compressor, for safety reasons.
			GetWatchdog().SetEnabled(false); // Teleop is done, so let's turn off Watchdog.
		}
	};

START_ROBOT_CLASS(Robot2011Crush);
