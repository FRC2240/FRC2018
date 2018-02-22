#include <iostream>
#include <string>
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
	double speedVariable = -0.8;
	Joystick *stick;

	WPI_TalonSRX *lFrontMotor;
	WPI_TalonSRX *lBackMotor;
	WPI_TalonSRX *rFrontMotor;
	WPI_TalonSRX *rBackMotor;

	WPI_TalonSRX *rElevator;
	WPI_TalonSRX *lElevator;

	WPI_TalonSRX *gatherer;
	VictorSP *gathererLifter;

	Servo *climbBarDeployer;
	DifferentialDrive *drive;

	Relay *ratchetSolenoid; //ratchet

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

	double kGrabberP;
	double kGrabberI;
	double kGrabberD;
	double kGrabberF;
	int    kGrabberOpenPosition;
	int    kGrabberClosedPosition;

	int    kAutoLinePosition;
	int    kAutoScalePositionStep1;
	int    kAutoScalePositionStep2;
	int    kAutoSwitchPositionStep1;
	int    kAutoSwitchPositionStep2;
	int    kAutoSwitchPositionStep3;
	double kAutoSwitchAngle;
	double kAutoSpeed;

	//DigitalInput *gathererLift; //0 - Limit switch detecting if the gatherer has been lifted fully
	//DigitalInput *gathererLimit;//1 - Limit switch detecting if the gatherer is fully extended inward

	enum levels{
		base,
		switchLevel,
		lowLever,
		highLever,
	};

//	enum Value{
//		kOff,
//		kOn,
//		kForward,
//		kReverse,
//	};
//	enum Direction{
//		kBothDirections,
//		kForwardOnly,
//		kReverseOnly,
//	};


	int currentLevel;
	int buttonTimer = 0;

	float deadBand(float value)
	{
		if(abs(value) < 0.1)
		{
			return 0.0;
		}
		else
		{
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

		kGrabberP = prefs->GetDouble("kGrabberP", 0.5);
		kGrabberI = prefs->GetDouble("kGrabberI", 0.0);
		kGrabberD = prefs->GetDouble("kGrabberD", 0.0);
		kGrabberF = prefs->GetDouble("kGrabberF", 0.0);
		kGrabberClosedPosition = prefs->GetInt("kGrabberClosedPosition", 0);
		kGrabberOpenPosition   = prefs->GetInt("kGrabberOpenPosition", 1000);

		kAutoLinePosition        = prefs->GetInt("kAutoLinePosition", 1000);
		kAutoScalePositionStep1  = prefs->GetInt("kAutoScalePositionStep1", 1000);
		kAutoScalePositionStep2  = prefs->GetInt("kAutoScalePositionStep2", 1000);
		kAutoSwitchPositionStep1 = prefs->GetInt("kAutoSwitchPositionStep1", 1000);
		kAutoSwitchPositionStep2 = prefs->GetInt("kAutoSwitchPositionStep2", 1000);
		kAutoSwitchPositionStep3 = prefs->GetInt("kAutoSwitchPositionStep3", 1000);
		kAutoSwitchAngle         = prefs->GetDouble("kAutoSwitchAngle", 0.0);
		kAutoSpeed               = prefs->GetDouble("kAutoSpeed", 0.0);

		lElevator->Config_kF(0, kElevatorF, 0);
		lElevator->Config_kP(0, kElevatorP, 0);
		lElevator->Config_kI(0, kElevatorI, 0);
		lElevator->Config_kD(0, kElevatorD, 0);
		rElevator->Config_kF(0, kElevatorF, 0);
		rElevator->Config_kP(0, kElevatorP, 0);
		rElevator->Config_kI(0, kElevatorI, 0);
		rElevator->Config_kD(0, kElevatorD, 0);

		gatherer->Config_kF(0, kGrabberF, 0);
		gatherer->Config_kP(0, kGrabberP, 0);
		gatherer->Config_kI(0, kGrabberI, 0);
		gatherer->Config_kD(0, kGrabberD, 0);
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

		gatherer  = new WPI_TalonSRX(8);
		gatherer ->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		gatherer->SelectProfileSlot(0, 0);

		gatherer->Config_kF(0, 0.0, 0);
		gatherer->Config_kP(0, 0.2, 0);
		gatherer->Config_kI(0, 0.0, 0);
		gatherer->Config_kD(0, 0.0, 0);
		gatherer->ConfigAllowableClosedloopError(0, 0, 0);


		gathererLifter = new VictorSP(9);

//		gathererLift = new DigitalInput(0);
//		gathererLimit = new DigitalInput(1);

//		boopieSolenoid = new frc::Relay(3,frc::Relay::kReverseOnly);
		ratchetSolenoid = new frc::Relay(0,frc::Relay::kReverseOnly);

//		boopieSolenoid-> Set(Relay::kOff);
		ratchetSolenoid-> Set(Relay::kOff);

	}

	void AutonomousInit() override {
		getPreferences();

		rFrontMotor->SetSelectedSensorPosition(0, 0, 0);
		lFrontMotor->SetSelectedSensorPosition(0, 0, 0);

	}

	void AutonomousPeriodic() {
		bool Done = false;
		if(Done == false){
			lFrontMotor->Set(ControlMode::Position, 25077);
			rFrontMotor->Set(ControlMode::Position, 25077);
			Done = true;
}
//		else{
//			lFrontMotor->Set(ControlMode::Velocity, 0.0);
//			rFrontMotor->Set(ControlMode::Velocity, 0.0);
//}


	}

	void TeleopInit() {
		currentLevel = base;
		getPreferences();
	}

	void TeleopPeriodic() {

		drive->ArcadeDrive(speedVariable * stick->GetRawAxis(1), stick->GetRawAxis(4));
		//LOGGER(INFO) << lElevator->Get();
		//lElevator->Set(ControlMode::Position, -3*4096);
		//rElevator->Set(ControlMode::Position, 3*4096);
		//		if(stick->GetPOV() == 90) {
		//			lElevator->Set(ControlMode::Position, -4000);
		//			rElevator->Set(ControlMode::Position, 4000);
		//		} else if(stick->GetPOV() == 270) {
		//			lElevator->Set(ControlMode::Position, -250);
		//			rElevator->Set(ControlMode::Position, 250);
		//		}

		LOGGER(INFO) << "Positions: " << rElevator->GetSelectedSensorPosition(0) << " " << lElevator->GetSelectedSensorPosition(0);

		if((stick->GetRawAxis(3) > 0.1) && buttonTimer > 50){//34777u 34500 = top
			lElevator->Set(ControlMode::Position, 20000);
			rElevator->Set(ControlMode::Position, 20000);
			LOGGER(INFO) << "Right Trigger";
			//currentLevel++;
			buttonTimer = 0;
			speedVariable = -0.4;
		}
		else if((stick->GetRawAxis(2) > 0.1) && buttonTimer > 50){
			//currentLevel--;
			buttonTimer = 0;
			LOGGER(INFO) << "Left Trigger";
			lElevator->Set(ControlMode::Position, 250);
			rElevator->Set(ControlMode::Position, 250);
			speedVariable = -0.8;
		}

		buttonTimer++;
//		if(currentLevel == base)
//		{
//			lElevator->Set(ControlMode::Position, -250);
//			rElevator->Set(ControlMode::Position, 250);
//
//		}
//		else if(currentLevel == middle)
//		{
//			lElevator->Set(ControlMode::Position, -4000);
//			rElevator->Set(ControlMode::Position, 4000);
//		}
//		else if(currentLevel == top)
//		{
//			lElevator->Set(ControlMode::Position, -17750);
//			rElevator->Set(ControlMode::Position, 17750);
//		}
//
		//LOGGER(INFO) << stick->GetPOV();

		if(stick->GetRawButton(4))
		{
			gathererLifter->Set(1.0);
		}
		else if(stick->GetRawButton(1))
		{
			gathererLifter->Set(-1.0);
		}
		else
		{
			gathererLifter->Set(0.0);
		}

//		if(stick->GetRawButton(6))
//		{
//			gatherer->Set(1.0);
//		}
//		else if(stick->GetRawButton(5))
//		{
//			gatherer->Set(-1.0);
//		}
//		else
//		{
//			gatherer->Set(0.0);
		if(stick->GetRawButton(6) && buttonTimer > 50){
			gatherer->Set(ControlMode::Position, 2000);
			//LOGGER(INFO) << "Pressed 90";
			//currentLevel++;
			buttonTimer = 0;
				}
		else if(stick->GetRawButton(5) && buttonTimer > 50){
			//currentLevel--;
			buttonTimer = 0;
			//LOGGER(INFO) << "Pressed 270";
			gatherer->Set(ControlMode::Position, 400);
		}

		/*if(!gathererLift->Get() && stick->GetRawButton(4))
		{
			gathererLifter->Set(ControlMode::PercentOutput, 0.3);
		}
		else if(stick->GetRawButton(1))
		{
			gathererLifter->Set(ControlMode::PercentOutput, -0.3);
		}
		else
		{
			gathererLifter->Set(ControlMode::PercentOutput, 0.0);
			if(stick->GetRawButton(2) && !gathererLimit->Get())
			{
				gatherer->Set(ControlMode::PercentOutput, -0.3);
			}
			else if(stick->GetRawButton(3))
			{
				gatherer->Set(ControlMode::PercentOutput, 0.3);
			}
			else
			{
				gatherer->Set(ControlMode::PercentOutput, 0.0);
			}
		}*/

		//Toggles on Boopie Solenoid, unlatches bar for other robots- is "start" button on controller
//		if(stick->GetRawButton(8) == 1){
//			LOGGER(INFO) << "boopieSolenoid";
//			if(boopieSolenoid->Get() == 0){
//				boopieSolenoid->Set(frc::Relay::kOn);
//			}
//		}
//			else if(stick->GetRawButton(8) == 0 && boopieSolenoid->Get() == 1){
//				boopieSolenoid->Set(frc::Relay::kOff);
//			}
//			buttonTimer++;
		if(stick->GetRawButton(7) == 1){
			LOGGER(INFO) << "doopieSolenoid";
			if(ratchetSolenoid->Get() == 0){
				ratchetSolenoid->Set(frc::Relay::kOn);
			}
		}
			else if(stick->GetRawButton(7) == 0 && ratchetSolenoid->Get() == 1){
				ratchetSolenoid->Set(frc::Relay::kOff);
			}


		if(stick->GetRawButton(8) == 1){
			climbBarDeployer->Set(0.0);
		}
		else{
			climbBarDeployer->Set(0.0);
		}
	if(stick->GetRawButton(3	)){//34777u 34500 = top
		lElevator->Set(ControlMode::Position, kClimbStart);
		rElevator->Set(ControlMode::Position, kClimbStart);
		LOGGER(INFO) << "climber activated";
		//currentLevel++;
	}
	else if(stick->GetRawButton(2)){
		//currentLevel--;
		LOGGER(INFO) << "climbing...";
		lElevator->Set(ControlMode::Position, kClimbStop);
		rElevator->Set(ControlMode::Position, kClimbStop);

	}

	}

	void TestPeriodic(){}
};

START_ROBOT_CLASS(Robot)
