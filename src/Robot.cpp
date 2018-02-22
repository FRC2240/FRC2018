#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <WPIlib.h>
#include <log.h>
#include <ctre/Phoenix.h>
#include <Solenoid.h>
#include <Relay.h>

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

	Preferences *prefs;

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
	double kAutoSpeed;

	// Elevator Levels
	enum levels {
		baseLevel    = 0,
		switchLevel  = 90,
		scaleLevelLo = 180,
		scaleLevelHi = 270,
	};

	// Autonomous Modes
	enum autoModes {
		autoLine,	// Just drive forward and cross the Auto Line
		autoSwitch,	// Place the cube on the Switch
		autoScale	// Place the Cube on the Scale
	};

	// Autonomous Functions
	enum autoStates {
		autoStateDriveOnBearing,	// Drive a distance on a gyro bearing
		autoStateTurning,			// Turn a set number of degrees
		autoStateRaiseCube,		// Raise the Cube to a set height
		autoStateReleaseCube,		// Release the Cube
		autoStateDone				// Complete
	};

	enum autoStateStatus {
		autoInProgress,
		autoSuccess,
		autoFail
	};

	// Autonomous functions
	autoStateStatus autoDriveOnBearing(double angle, double distance, int height, int timeout) {
		lFrontMotor->Set(ControlMode::Position, distance);
		rFrontMotor->Set(ControlMode::Position, distance);
		// TODO
		return autoSuccess;
	}
	autoStateStatus autoTurn(double angle, double distance, int height, int timeout) {
		// TODO
		return autoSuccess;
	}
	autoStateStatus autoRaiseCube(double angle, double distance, int height, int timeout) {
		// TODO
		return autoSuccess;
	}
	autoStateStatus autoReleaseCube(double angle, double distance, int height, int timeout) {
		// TODO
		return autoSuccess;
	}

	void autoStop() {
		// TODO
	}

	class autoFunction {
		autoFunction(autoStateStatus (*func)(double, int, int, int), double angle, int distance, int height, int timeout) :
			m_func(func), m_angle(angle), m_distance(distance), m_height(height), m_timeout(timeout) {}

		autoStateStatus call() { return m_func(m_angle, m_distance, m_height, m_timeout); }
		int timeout() { return m_timeout; }
	private:
		bool (*m_func)(double, int, int, int);
		double m_angle;
		int m_distance;
		int m_height;
		int m_timeout;
	};

	// Drive to Auto Line
	std::list<autoFunction> autoLineStates = {autoFunction(autoDriveOnBearing, 0.0, kAutoLinePosition, 0, 200)};

	// Deliver Cube to Switch
	std::list<autoFunction> autoSwitchStates = {
		autoFunction(autoDriveOnBearing, 0.0, kAutoSwitchPositionStep1, 0, 200),
		autoFunction(autoTurn, kAutoSwitchDriveAngle, 0, 0, 200),
		autoFunction(autoDriveOnBearing, kAutoSwitchDriveAngle, kAutoSwitchPositionStep2, 0, 200),
		autoFunction(autoTurn, 0.0, 0, 0, 200),
		autoFunction(autoRaiseCube, 0.0, 0, kElevatorSwitchPosition, 200),
		autoFunction(autoDriveOnBearing, 0.0, kAutoSwitchPositionStep3, 0, 200),
		autoFunction(autoReleaseCube, 0.0, 0, kElevatorSwitchPosition, 200)
	};

	// Deliver Cube to Scale
	std::list<autoFunction> autoScaleStates = {
		autoFunction(autoDriveOnBearing, 0.0, kAutoScalePositionStep1, 0, 200),
		autoFunction(autoTurn, 90.0, 0, 0, 200),
		autoFunction(autoRaiseCube, 0.0, 0, kElevatorScaleHighPosition, 200),
		autoFunction(autoDriveOnBearing, 0.0, kAutoScalePositionStep2, 0, 200),
		autoFunction(autoReleaseCube, 0.0, 0, kElevatorScaleHighPosition, 200)
	};

	std::list<autoFunction> autoStates;

	int autoTimer = 0;
	bool lastRatchetAction = false;

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

		kDriveP = prefs->GetDouble("kDriveP", 0.0);
		kDriveI = prefs->GetDouble("kDriveI", 0.0);
		kDriveD = prefs->GetDouble("kDriveD", 0.0);
		kDriveF = prefs->GetDouble("kDriveF", 0.0);

		kClawP = prefs->GetDouble("kClawP", 0.5);
		kClawI = prefs->GetDouble("kClawI", 0.0);
		kClawD = prefs->GetDouble("kClawD", 0.0);
		kClawF = prefs->GetDouble("kClawF", 0.0);
		kClawClosedPosition = prefs->GetInt("kClawClosedPosition", 0);
		kClawOpenPosition   = prefs->GetInt("kClawOpenPosition", 1000);

		kAutoLinePosition        = prefs->GetInt("kAutoLinePosition", 1000);
		kAutoScalePositionStep1  = prefs->GetInt("kAutoScalePositionStep1", 1000);
		kAutoScalePositionStep2  = prefs->GetInt("kAutoScalePositionStep2", 1000);
		kAutoSwitchPositionStep1 = prefs->GetInt("kAutoSwitchPositionStep1", 1000);
		kAutoSwitchPositionStep2 = prefs->GetInt("kAutoSwitchPositionStep2", 1000);
		kAutoSwitchPositionStep3 = prefs->GetInt("kAutoSwitchPositionStep3", 1000);
		kAutoSwitchDriveAngle    = prefs->GetDouble("kAutoSwitchAngle", 0.0);
		kAutoSpeed               = prefs->GetDouble("kAutoSpeed", 0.0);

		claw->Config_kF(0, kClawF, 0);
		claw->Config_kP(0, kClawP, 0);
		claw->Config_kI(0, kClawI, 0);
		claw->Config_kD(0, kClawD, 0);
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
		lElevator->ConfigForwardSoftLimitThreshold(-17750, 10);
		//lElevator->ConfigReverseSoftLimitThreshold(5*4096, 10);
		//lElevator->ConfigForwardSoftLimitEnable(true, 10);
		//lElevator->ConfigReverseSoftLimitEnable(true, 10);
		//lElevator->ConfigPeakOutputForward(0.25, 0);
		//lElevator->ConfigPeakOutputReverse(-0.25, 0);
		lElevator->SelectProfileSlot(0, 0);
		lElevator->ConfigAllowableClosedloopError(0, 0, 0);
		lElevator->SetSensorPhase(true);

		rElevator = new WPI_TalonSRX(5);
		rElevator->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		//rElevator->ConfigForwardSoftLimitThreshold(17750, 10);
		//rElevator->ConfigReverseSoftLimitThreshold(5*4096, 10);
		//rElevator->ConfigForwardSoftLimitEnable(true, 10);
		//rElevator->ConfigReverseSoftLimitEnable(true, 10);
		//rElevator->ConfigPeakOutputForward(0.75, 10);
		//rElevator->ConfigPeakOutputReverse(-0.75, 10);
		rElevator->SelectProfileSlot(0, 0);
		rElevator->ConfigAllowableClosedloopError(0, 0, 0);
		// Zero position
		rElevator->SetSelectedSensorPosition(0, 0, 0);
		lElevator->SetSelectedSensorPosition(0, 0, 0);

		currentLevel = base;

		claw  = new WPI_TalonSRX(8);
		claw ->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		claw->SelectProfileSlot(0, 0);
		claw->ConfigAllowableClosedloopError(0, 0, 0);
		claw->SetSelectedSensorPosition(0, 0, 0);

		clawLifter = new VictorSP(9);

		ratchetSolenoid = new frc::Relay(0,frc::Relay::kReverseOnly);
		ratchetSolenoid-> Set(Relay::kOff);
	}

	void AutonomousInit() override {
		getPreferences();

		// reset drive position
		rFrontMotor->SetSelectedSensorPosition(0, 0, 0);
		lFrontMotor->SetSelectedSensorPosition(0, 0, 0);

		climbBarDeployer->Set(kServoStart);

		autoTimer = 0;

		// TODO: Read FMS data
		// TODO: Read Starting Position (C, R, L)
		autoStates = autoSwitchStates;
	}

	void AutonomousPeriodic() {
		autoStateStatus status;
		++autoTimer;

		if (autoStates.empty()) {
			// Nothing left to do
			return;
		}

		// Call current auto function, if timeout not exceeded
		if (autoTimer < autoStates.front().timeout()) {
			status = autoStates.front().call();
		} else {
			// Timeout!
			autoStates.clear();
			autoStop();
			return;
		}

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
				autoStop();
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

		// TODO: Elevator Continuous Up/Down

		// Set elevator level
		if (elevatorLevelAction == levels::baseLevel) {
			enableElevatorPID();
			lElevator->Set(ControlMode::Position, 0);
			rElevator->Set(ControlMode::Position, 0);
			speedVariable = -1.0
			LOGGER(INFO) << "ACTION: SET ELEVATOR TO BASE LEVEL";
		} else if (elevatorLevelAction == levels::switchLevel) {
			enableElevatorPID();
			lElevator->Set(ControlMode::Position, kElevatorSwitchPosition);
			rElevator->Set(ControlMode::Position, kElevatorSwitchPosition);
			speedVariable = -0.8
			LOGGER(INFO) << "ACTION: SET ELEVATOR TO SWITCH LEVEL";
		} else if (elevatorLevelAction == levels::scaleLevelLo) {
			enableElevatorPID();
			lElevator->Set(ControlMode::Position, kElevatorScaleLowPosition);
			rElevator->Set(ControlMode::Position, kElevatorScaleLowPosition);
			speedVariable = -0.4
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

			lElevator->Set(ControlMode::Position, 0);
			rElevator->Set(ControlMode::Position, 0);
			LOGGER(INFO) << "ACTION: CLIMBING";
		}
	}

	void TestPeriodic() {
		// TODO: PUT constants
	}
};

START_ROBOT_CLASS(Robot)
