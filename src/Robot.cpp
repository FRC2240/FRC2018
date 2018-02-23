#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <WPIlib.h>
#include <log.h>
#include <ctre/Phoenix.h>
#include <Solenoid.h>
#include <Relay.h>
#include "AHRS.h"
#include <SmartDashboard/SmartDashboard.h>

//4096:1 revolution for encoder
// 1st preference: raising (R) lowering (L) for triggers
// 2nd pref: bumpers for open (R) and closing (L)
// lower close (a,b, x or y)
// wheel diameter = 6.25 in   Circumference = 19.63495408493620 inches.

class Robot : public frc::IterativeRobot {
private:
	double speedVariable = -1.0;
	Joystick *stick;

	WPI_TalonSRX *lFrontMotor;
	WPI_TalonSRX *lBackMotor;
	WPI_TalonSRX *rFrontMotor;
	WPI_TalonSRX *rBackMotor;

	WPI_TalonSRX *rElevator;
	WPI_TalonSRX *lElevator;

	WPI_TalonSRX *claw;
	VictorSP     *clawLifter;

	Servo *climbBarDeployer;
	DifferentialDrive *drive;

	Relay *ratchetSolenoid;
	AHRS  *ahrs; // navX MXP
	Preferences *prefs;

	std::string autoStartPosition;
	const std::string autoNameDefault = "center";
	int autoTimer = 0;
	bool lastRatchetAction = false;

	char switchPosition = 'U';
	char scalePosition  = 'U';

	enum autoStateStatus {
		autoInProgress,
		autoSuccess,
		autoFail
	};

	// Class to wrap an autonomous function and it's parameters to make it serialize-able
	class autoFunction {
	public:
		autoFunction(autoStateStatus (Robot::*func)(double, int, int, int), double angle, int distance, int height, int timeout, int timer) :
			m_func(func), m_angle(angle), m_distance(distance), m_height(height), m_timeout(timeout) {}

		autoStateStatus call(Robot& obj, int timer) { return ((obj).*(m_func))(m_angle, m_distance, m_height, m_timeout, timer); }
		int timeout() { return m_timeout; }
	private:
		autoStateStatus (Robot::*m_func)(double, int, int, int, int);
		double m_angle;
		int m_distance;
		int m_height;
		int m_timeout;
	};

	std::list<autoFunction> autoStates;

	/***** Tunable constants *****/
	double kServoStart;
	double kServoStop;

	double kElevatorP;
	double kElevatorI;
	double kElevatorD;
	double kElevatorF;
	int    kElevatorHighLimit;
	int    kElevatorSwitchPosition;
	int    kElevatorScaleLowPosition;
	int    kElevatorScaleHighPosition;
	int    kElevatorClimbStopPosition;
	int    kElevatorDistancePerTick;

	double kDriveP;
	double kDriveI;
	double kDriveD;
	double kDriveF;

	double kTurnP;
	double kTurnI;
	double kTurnD;
	double kTurnF;
	double kTurnOutputRange;
	double kTurnToleranceDegrees;

	double kClawP;
	double kClawI;
	double kClawD;
	double kClawF;
	int    kClawOpenPosition;
	int    kClawClosedPosition;

	int    kAutoLinePosition;
	int    kAutoScalePositionStep1;
	int    kAutoScalePositionStep2;
	int    kAutoSwitchPositionStep1;
	int    kAutoSwitchPositionStep2;
	int    kAutoSwitchPositionStep3;
	double kAutoSwitchDriveAngle;
	double kAutoScaleDriveAngle;
	double kAutoSpeed;

	// Elevator Levels
	enum levels {
		baseLevel    = 0,
		switchLevel  = 90,
		scaleLevelLo = 180,
		scaleLevelHi = 270,
	};

	PIDController *turnController = NULL;    // PID Controller

	// Output from the Turn (Angle) PID Controller
	class TurnPIDOutput : public PIDOutput {
	public:
		double correction = 0.0;
		void PIDWrite(double output) {
			correction = output;
		}
	} turnPIDOutput;

	// Autonomous functions
	autoStateStatus autoDriveOnBearing(double angle, int distance, int height, int timeout, int timer) {
		lFrontMotor->Set(ControlMode::Position, distance);
		rFrontMotor->Set(ControlMode::Position, distance);

		double avg = (lFrontMotor->GetSelectedSensorPosition(0) + rFrontMotor->GetSelectedSensorPosition(0))/2.0;
		double diff = fabs(double(distance) - avg);
		if (diff < 1000.0) {
			return autoSuccess;
		} else if (timer > timeout) {
			return autoFail;
		} else {
			return autoInProgress;
		}
	}

	autoStateStatus autoTurn(double angle, int distance, int height, int timeout, int timer) {
		turnController->SetSetpoint(angle);
		turnController->Enable();
		drive->ArcadeDrive(0.0, -turnPIDOutput.correction);

		if ((ahrs->GetAngle() - angle) < kTurnToleranceDegrees) {
			return autoSuccess;
		} else if (timer > timeout) {
			return autoFail;
		} else {
			return autoInProgress;
		}
	}

	autoStateStatus autoRaiseCube(double angle, int distance, int height, int timeout, int timer) {
		// TODO
		return autoSuccess;
	}

	autoStateStatus autoReleaseCube(double angle, int distance, int height, int timeout, int timer) {
		// TODO
		return autoSuccess;
	}

	// Drive to Auto Line
	std::list<autoFunction> createAutoLineStates () {
		return {autoFunction(&Robot::autoDriveOnBearing, 0.0, kAutoLinePosition, 0, 200)};
	}

	// Deliver Cube to Switch
	std::list<autoFunction> createAutoSwitchStates () {
		return {
		autoFunction(&Robot::autoDriveOnBearing, 0.0, kAutoSwitchPositionStep1, 0, 200),
		autoFunction(&Robot::autoTurn, kAutoSwitchDriveAngle, 0, 0, 200),
		autoFunction(&Robot::autoDriveOnBearing, kAutoSwitchDriveAngle, kAutoSwitchPositionStep2, 0, 200),
		autoFunction(&Robot::autoTurn, 0.0, 0, 0, 200),
		autoFunction(&Robot::autoRaiseCube, 0.0, 0, kElevatorSwitchPosition, 200),
		autoFunction(&Robot::autoDriveOnBearing, 0.0, kAutoSwitchPositionStep3, 0, 200),
		autoFunction(&Robot::autoReleaseCube, 0.0, 0, kElevatorSwitchPosition, 200)
		};
	}

	// Deliver Cube to Scale
	std::list<autoFunction> createAutoScaleStates() {
		return {
		autoFunction(&Robot::autoDriveOnBearing, 0.0, kAutoScalePositionStep1, 0, 200),
		autoFunction(&Robot::autoTurn, kAutoScaleDriveAngle, 0, 0, 200),
		autoFunction(&Robot::autoRaiseCube, 0.0, 0, kElevatorScaleHighPosition, 200),
		autoFunction(&Robot::autoDriveOnBearing, kAutoScaleDriveAngle, kAutoScalePositionStep2, 0, 200),
		autoFunction(&Robot::autoReleaseCube, 0.0, 0, kElevatorScaleHighPosition, 200)
		};
	}

	// Dead-band filter
	float deadBand(double value)
	{
		if (fabs(value) < 0.1) {
			return 0.0;
		} else {
			return value;
		}
	}

	// Read data from the Preferences Panel
	void getPreferences()
	{
		kServoStart = prefs->GetDouble("kServoStart", 0.5);
		kServoStop  = prefs->GetDouble("kServoStop", 0.8);

		kElevatorP = prefs->GetDouble("kElevatorP", 1.0);
		kElevatorI = prefs->GetDouble("kElevatorI", 0.0);
		kElevatorD = prefs->GetDouble("kElevatorD", 0.0);
		kElevatorF = prefs->GetDouble("kElevatorF", 0.0);

		kElevatorHighLimit         = prefs->GetInt("kElevatorHighLimit", 30000);
		kElevatorSwitchPosition    = prefs->GetInt("kElevatorSwitchPosition", 8000);
		kElevatorScaleLowPosition  = prefs->GetInt("kElevatorScaleLowPosition", 20000);
		kElevatorScaleHighPosition = prefs->GetInt("kElevatorScaleHighPosition", 27000);
		kElevatorClimbStopPosition = prefs->GetInt("kElevatorClimbPosition", 8000);
		kElevatorDistancePerTick   = prefs->GetInt("kElevatorDistancePerTick", 400);

		kDriveP = prefs->GetDouble("kDriveP", 1.0);
		kDriveI = prefs->GetDouble("kDriveI", 0.0);
		kDriveD = prefs->GetDouble("kDriveD", 0.0);
		kDriveF = prefs->GetDouble("kDriveF", 0.0);

		kClawP = prefs->GetDouble("kClawP", 0.5);
		kClawI = prefs->GetDouble("kClawI", 0.0);
		kClawD = prefs->GetDouble("kClawD", 0.0);
		kClawF = prefs->GetDouble("kClawF", 0.0);
		kClawClosedPosition = prefs->GetInt("kClawClosedPosition", 0);
		kClawOpenPosition   = prefs->GetInt("kClawOpenPosition", 1000);

		kAutoLinePosition        = prefs->GetInt("kAutoLinePosition", 37000);			// 9 revs
		kAutoScalePositionStep1  = prefs->GetInt("kAutoScalePositionStep1", 70000);		// 17 revs
		kAutoScalePositionStep2  = prefs->GetInt("kAutoScalePositionStep2", 2000);		// 0.5 rev
		kAutoScaleDriveAngle     = prefs->GetDouble("kAutoScaleDriveAngle", 90.0);
		kAutoSwitchPositionStep1 = prefs->GetInt("kAutoSwitchPositionStep1", 4100);		// 1 rev
		kAutoSwitchPositionStep2 = prefs->GetInt("kAutoSwitchPositionStep2", 18500);	// 4.5 rvs
		kAutoSwitchPositionStep3 = prefs->GetInt("kAutoSwitchPositionStep3", 10200);	// 2.5 revs
		kAutoSwitchDriveAngle    = prefs->GetDouble("kAutoSwitchDriveAngle", 45.0);
		kAutoSpeed               = prefs->GetDouble("kAutoSpeed", 0.6);

		kTurnP                = prefs->GetDouble("kTurnP", 0.2);
		kTurnI                = prefs->GetDouble("kTurnI", 0.0);
		kTurnD                = prefs->GetDouble("kTurnD", 0.2);
		kTurnF                = prefs->GetDouble("kTurnF", 0.0);
		kTurnOutputRange      = prefs->GetDouble("kTurnOutputRange", 0.2);
		kTurnToleranceDegrees = prefs->GetDouble("kTurnToleranceDegrees", 0.1);

		claw->Config_kF(0, kClawF, 0);
		claw->Config_kP(0, kClawP, 0);
		claw->Config_kI(0, kClawI, 0);
		claw->Config_kD(0, kClawD, 0);

		lElevator->ConfigForwardSoftLimitEnable(true, 0);
		lElevator->ConfigReverseSoftLimitEnable(true, 0);
		lElevator->ConfigForwardSoftLimitThreshold(kElevatorHighLimit, 0);
		lElevator->ConfigReverseSoftLimitThreshold(0, 0);
		rElevator->ConfigForwardSoftLimitEnable(true, 0);
		rElevator->ConfigReverseSoftLimitEnable(true, 0);
		rElevator->ConfigForwardSoftLimitThreshold(kElevatorHighLimit, 0);
		rElevator->ConfigReverseSoftLimitThreshold(0, 0);
	}

	void disableElevatorPID() {
		lElevator->Config_kF(0, 0.0, 0);
		lElevator->Config_kP(0, 0.0, 0);
		lElevator->Config_kI(0, 0.0, 0);
		lElevator->Config_kD(0, 0.0, 0);
		rElevator->Config_kF(0, 0.0, 0);
		rElevator->Config_kP(0, 0.0, 0);
		rElevator->Config_kI(0, 0.0, 0);
		rElevator->Config_kD(0, 0.0, 0);
	}

	void enableElevatorPID() {
		lElevator->Config_kF(0, kElevatorF, 0);
		lElevator->Config_kP(0, kElevatorP, 0);
		lElevator->Config_kI(0, kElevatorI, 0);
		lElevator->Config_kD(0, kElevatorD, 0);
		rElevator->Config_kF(0, kElevatorF, 0);
		rElevator->Config_kP(0, kElevatorP, 0);
		rElevator->Config_kI(0, kElevatorI, 0);
		rElevator->Config_kD(0, kElevatorD, 0);
	}

public:

	void RobotInit() {
		prefs = Preferences::GetInstance();

		stick = new Joystick(0);

		climbBarDeployer = new Servo(5);

		lFrontMotor = new WPI_TalonSRX(4);
		lFrontMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);

		lBackMotor = new WPI_TalonSRX(3);
		lBackMotor->Set(ControlMode::Follower, 4);

		rFrontMotor = new WPI_TalonSRX(2);
		rFrontMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);

		rBackMotor = new WPI_TalonSRX(1);
		rBackMotor->Set(ControlMode::Follower, 2);

		drive = new DifferentialDrive(*lFrontMotor, *rFrontMotor);

		lElevator = new WPI_TalonSRX(6);
		lElevator->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		//lElevator->ConfigPeakOutputForward(0.25, 0);
		//lElevator->ConfigPeakOutputReverse(-0.25, 0);
		lElevator->SelectProfileSlot(0, 0);
		lElevator->ConfigAllowableClosedloopError(0, 0, 0);
		lElevator->SetSensorPhase(true);

		rElevator = new WPI_TalonSRX(5);
		rElevator->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		//rElevator->ConfigPeakOutputForward(0.75, 10);
		//rElevator->ConfigPeakOutputReverse(-0.75, 10);
		rElevator->SelectProfileSlot(0, 0);
		rElevator->ConfigAllowableClosedloopError(0, 0, 0);

		// Zero position
		rElevator->SetSelectedSensorPosition(0, 0, 0);
		lElevator->SetSelectedSensorPosition(0, 0, 0);

		claw = new WPI_TalonSRX(8);
		claw ->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		claw->SelectProfileSlot(0, 0);
		claw->ConfigAllowableClosedloopError(0, 0, 0);
		claw->SetSelectedSensorPosition(0, 0, 0);

		clawLifter = new VictorSP(9);

		ratchetSolenoid = new frc::Relay(0,frc::Relay::kReverseOnly);
		ratchetSolenoid-> Set(Relay::kOff);

		try {
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception& ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
	}

	void AutonomousInit() override {
		getPreferences();

		turnController = new PIDController(kTurnP, kTurnI, kTurnD, kTurnF, ahrs, &turnPIDOutput);
		turnController->SetInputRange(-180.0, 180.0);
		turnController->SetOutputRange(-kTurnOutputRange, kTurnOutputRange);
		turnController->SetAbsoluteTolerance(kTurnToleranceDegrees);
		turnController->SetContinuous(true);

		// reset drive position
		rFrontMotor->SetSelectedSensorPosition(0, 0, 0);
		lFrontMotor->SetSelectedSensorPosition(0, 0, 0);

		climbBarDeployer->Set(kServoStart);

		ahrs->ZeroYaw();

		autoTimer = 0;

		// Read Game Data from FMS
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData[0] == 'L') {
				switchPosition = 'L';
			} else {
				switchPosition = 'R';
			}
			if (gameData[1] == 'L') {
				scalePosition = 'L';
			} else {
				scalePosition = 'R';
			}
		}

		// Read starting position from Driver Station
		autoStartPosition = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		LOGGER(INFO) << "[AutoInit] Auto Start Position: " << autoStartPosition;

		// Determine which Autonomous Mode to run
		if (autoStartPosition.find("R") != std::string::npos) {
			if (scalePosition == 'R') {
				// flip angle
				kAutoScaleDriveAngle = -kAutoScaleDriveAngle;
				autoStates = createAutoScaleStates();
				LOGGER(INFO) << "[AutoInit] AUTO SCALE MODE, RIGHT";
			} else {
				autoStates = createAutoLineStates();
				LOGGER(INFO) << "[AutoInit] AUTO LINE MODE, RIGHT";
			}
		} else if (autoStartPosition.find("L") != std::string::npos){
			if (scalePosition == 'L') {
				autoStates = createAutoScaleStates();
				LOGGER(INFO) << "[AutoInit] AUTO SCALE MODE, LEFT";
			} else {
				autoStates = createAutoLineStates();
				LOGGER(INFO) << "[AutoInit] AUTO LINE MODE, LEFT";
			}
		} else {
			if (switchPosition == 'L') {
				// flip angle
				kAutoSwitchDriveAngle = -kAutoSwitchDriveAngle;
				LOGGER(INFO) << "[AutoInit] AUTO SWITCH MODE, LEFT";
			} else {
				LOGGER(INFO) << "[AutoInit] AUTO SWITCH MODE, RIGHT";
			}
			autoStates = createAutoSwitchStates();
		}
	}

	void AutonomousPeriodic() {
		++autoTimer;

		if (autoStates.empty()) {
			// Nothing left to do
			return;
		}

		// Call current auto function
		autoStateStatus status = autoStates.front().call(*this, autoTimer);

		// Check status
		switch (status) {
			case autoInProgress:
				// Keep going!
				break;
			case autoSuccess:
				// Go to next state
				autoStates.pop_front();
				autoTimer = 0;
				break;
			case autoFail:
				// Something went wrong, stop
				autoStates.clear();
				break;
		}
	}

	void TeleopInit() {
		getPreferences();
	}

	void TeleopPeriodic() {
        // Joystick inputs
		double driveForwardAction  = stick->GetRawAxis(1);
		double driveTurnAction     = stick->GetRawAxis(4);
		double elevatorDownAction  = stick->GetRawAxis(2);
		double elevatorUpAction    = stick->GetRawAxis(3);
		bool   climbAction         = stick->GetRawButton(2);
		bool   clawUpAction        = stick->GetRawButton(1);
		bool   clawDownAction      = stick->GetRawButton(4);
		bool   clawOpenAction      = stick->GetRawButton(5);
		bool   clawCloseAction     = stick->GetRawButton(6);
		bool   ratchetAction       = stick->GetRawButton(7);
		bool   servoAction         = stick->GetRawButton(8);
		int    elevatorLevelAction = stick->GetPOV();

		// robot drive
		drive->ArcadeDrive(speedVariable * driveForwardAction, driveTurnAction);

		// Continuous elevator
		if (elevatorUpAction && !elevatorDownAction) {
			int rpos = kElevatorDistancePerTick + rElevator->GetSelectedSensorPosition(0);
			int lpos = kElevatorDistancePerTick + lElevator->GetSelectedSensorPosition(0);

			lElevator->Set(ControlMode::Position, lpos);
			rElevator->Set(ControlMode::Position, rpos);
		}

		if (elevatorDownAction && !elevatorUpAction) {
			int rpos = -kElevatorDistancePerTick + rElevator->GetSelectedSensorPosition(0);
			int lpos = -kElevatorDistancePerTick + lElevator->GetSelectedSensorPosition(0);

			lElevator->Set(ControlMode::Position, lpos);
			rElevator->Set(ControlMode::Position, rpos);
		}

		// Set elevator level
		if (elevatorLevelAction == levels::baseLevel) {
			enableElevatorPID();
			lElevator->Set(ControlMode::Position, 0);
			rElevator->Set(ControlMode::Position, 0);
			speedVariable = -1.0;
			LOGGER(INFO) << "ACTION: SET ELEVATOR TO BASE LEVEL";
		} else if (elevatorLevelAction == levels::switchLevel) {
			enableElevatorPID();
			lElevator->Set(ControlMode::Position, kElevatorSwitchPosition);
			rElevator->Set(ControlMode::Position, kElevatorSwitchPosition);
			speedVariable = -0.8;
			LOGGER(INFO) << "ACTION: SET ELEVATOR TO SWITCH LEVEL";
		} else if (elevatorLevelAction == levels::scaleLevelLo) {
			enableElevatorPID();
			lElevator->Set(ControlMode::Position, kElevatorScaleLowPosition);
			rElevator->Set(ControlMode::Position, kElevatorScaleLowPosition);
			speedVariable = -0.4;
			LOGGER(INFO) << "ACTION: SET ELEVATOR TO SCALE-LOW LEVEL";
		} else if (elevatorLevelAction == levels::scaleLevelHi) {
			enableElevatorPID();
			lElevator->Set(ControlMode::Position, kElevatorScaleHighPosition);
			rElevator->Set(ControlMode::Position, kElevatorScaleHighPosition);
			speedVariable = -0.4;
			LOGGER(INFO) << "ACTION: SET ELEVATOR TO SCALE-HIGH LEVEL";
		}

		LOGGER(INFO) << "Elevator Position: " << rElevator->GetSelectedSensorPosition(0) << " " << lElevator->GetSelectedSensorPosition(0);

		// Claw Up/Down
		if (clawDownAction && !clawUpAction) {
			clawLifter->Set(1.0);
			LOGGER(INFO) << "ACTION: CLAW DOWN";
		} else if (clawUpAction && !clawDownAction) {
			clawLifter->Set(-1.0);
			LOGGER(INFO) << "ACTION: CLAW UP";
		} else {
			clawLifter->Set(0.0);
		}

		// Claw Open/Close
		if (clawOpenAction && !clawCloseAction) {
			claw->Set(ControlMode::Position, kClawOpenPosition);
			LOGGER(INFO) << "ACTION: CLAW OPEN";
		}
		if (clawCloseAction && !clawOpenAction) {
			claw->Set(ControlMode::Position, kClawClosedPosition);
			LOGGER(INFO) << "ACTION: CLAW CLOSE";
		}

		// Toggles the solenoid ratchet
		if (ratchetAction && !lastRatchetAction) {
			if (ratchetSolenoid->Get() == frc::Relay::kOff){
				ratchetSolenoid->Set(frc::Relay::kOn);
				LOGGER(INFO) << "ACTION: RATCHET DISENGAGED";
			}
		    else if (ratchetSolenoid->Get() == frc::Relay::kOn){
				ratchetSolenoid->Set(frc::Relay::kOff);
				LOGGER(INFO) << "ACTION: RATCHET ENGAGED";
			}
		}
		lastRatchetAction = ratchetAction;

		// Deploy Climb Bar
		if (servoAction) {
			climbBarDeployer->Set(kServoStop);
			LOGGER(INFO) << "ACTION: DEPLOY CLIMB BAR";
		} else {
			climbBarDeployer->Set(kServoStart);
		}

		// Climb
		if (climbAction) {
			// Disable the ElevatorPID so that we don't accidentally hang using the motors
			disableElevatorPID();
			// Engage ratchet
			ratchetSolenoid->Set(frc::Relay::kOff);

			lElevator->Set(ControlMode::Position, 1000);
			rElevator->Set(ControlMode::Position, 1000);
			LOGGER(INFO) << "ACTION: CLIMBING";
		}
	}

	void TestPeriodic() {
		// Write to Preferences file, much faster than typing it all
		// by hand
		if (stick->GetRawButton(2) && stick->GetRawButton(3)) {
			 prefs->PutDouble("kServoStart", 0.5);
			 prefs->PutDouble("kServoStop", 0.8);

			 prefs->PutDouble("kElevatorP", 1.0);
			 prefs->PutDouble("kElevatorI", 0.0);
			 prefs->PutDouble("kElevatorD", 0.0);
			 prefs->PutDouble("kElevatorF", 0.0);
			 prefs->PutInt("kElevatorHighLimit", 30000);
			 prefs->PutInt("kElevatorSwitchPosition", 8000);
			 prefs->PutInt("kElevatorScaleLowPosition", 20000);
			 prefs->PutInt("kElevatorScaleHighPosition", 27000);
			 prefs->PutInt("kElevatorClimbPosition", 8000);
			 prefs->PutInt("kElevatorDistancePerTick", 400);

			 prefs->PutDouble("kDriveP", 0.0);
			 prefs->PutDouble("kDriveI", 0.0);
			 prefs->PutDouble("kDriveD", 0.0);
			 prefs->PutDouble("kDriveF", 0.0);

			 prefs->PutDouble("kClawP", 0.5);
			 prefs->PutDouble("kClawI", 0.0);
			 prefs->PutDouble("kClawD", 0.0);
			 prefs->PutDouble("kClawF", 0.0);
			 prefs->PutInt("kClawClosedPosition", 0);
			 prefs->PutInt("kClawOpenPosition", 1000);

			 prefs->PutInt("kAutoLinePosition", 1000);
			 prefs->PutInt("kAutoScalePositionStep1", 1000);
			 prefs->PutInt("kAutoScalePositionStep2", 1000);
			 prefs->PutDouble("kAutoScaleDriveAngle", 0.0);
			 prefs->PutInt("kAutoSwitchPositionStep1", 1000);
			 prefs->PutInt("kAutoSwitchPositionStep2", 1000);
			 prefs->PutInt("kAutoSwitchPositionStep3", 1000);
			 prefs->PutDouble("kAutoSwitchDriveAngle", 0.0);
			 prefs->PutDouble("kAutoSpeed", 0.0);
		}
	}
};

START_ROBOT_CLASS(Robot)
