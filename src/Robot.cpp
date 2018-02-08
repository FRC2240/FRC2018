#include <iostream>
#include <string>
#include <WPIlib.h>
#include <ctre/Phoenix.h>

Joystick *stick;

WPI_TalonSRX *lFrontMotor;
WPI_TalonSRX *lBackMotor;
WPI_TalonSRX *rFrontMotor;
WPI_TalonSRX *rBackMotor;

WPI_TalonSRX *rElevator;
WPI_TalonSRX *lElevator;

VictorSPX *gathererLifter;
VictorSPX *gatherer;

DifferentialDrive *drive;


class Robot : public frc::IterativeRobot {
public:
	float deadBand(float value)
	{
		if(abs(value) < .1)
		{
			return 0.0;
		}
		else
		{
			return value;
		}
	}

	void RobotInit() {
		stick = new Joystick(0);

		lFrontMotor = new WPI_TalonSRX(4);
		//lFrontMotor->SetSafetyEnabled(false);
		//lFrontMotor->Set(ControlMode::PercentOutput, 0.0);
		//lFrontMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
		//lFrontMotor->SetSensorPhase(true);

		lBackMotor = new WPI_TalonSRX(3);
		//lBackMotor->SetSafetyEnabled(false);
		//lBackMotor->Set(ControlMode::PercentOutput, 0.0);
		lBackMotor->Set(ControlMode::Follower, 4);
		//lBackMotor->SetInverted(true);

		rFrontMotor = new WPI_TalonSRX(2);
		//rFrontMotor->SetSafetyEnabled(false);
		//rFrontMotor->Set(ControlMode::PercentOutput, 0.0);
		//rFrontMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);

		rBackMotor = new WPI_TalonSRX(1);
		//rBackMotor->SetSafetyEnabled(false);
		//rBackMotor->Set(ControlMode::PercentOutput, 0.0);
		rBackMotor->Set(ControlMode::Follower, 2);
		//rBackMotor->SetInverted(true);

		drive = new DifferentialDrive(*lFrontMotor, *rFrontMotor);

		lElevator = new WPI_TalonSRX(5);

		rElevator = new WPI_TalonSRX(6);

		gatherer 	   = new VictorSPX(8);
		gathererLifter = new VictorSPX(9);



	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {}

	void TeleopPeriodic() {

		//drive->TankDrive(stick->GetRawAxis(1), stick->GetRawAxis(4));
		drive->ArcadeDrive(deadBand(-stick->GetRawAxis(1)), deadBand(.5*stick->GetRawAxis(4)));

		if(stick->GetRawButton(4)){
			lElevator->Set(ControlMode::PercentOutput, 0.25);
			rElevator->Set(ControlMode::PercentOutput, -0.25);
		}
		else if(stick->GetRawButton(1)){
			lElevator->Set(ControlMode::PercentOutput, -0.25);
			rElevator->Set(ControlMode::PercentOutput, 0.25);
		}
		else{
			lElevator->Set(ControlMode::PercentOutput, 0.0);
			rElevator->Set(ControlMode::PercentOutput, 0.0);
		}

		if(stick->GetRawButton(2))
		{
			gathererLifter->Set(ControlMode::PercentOutput, 1.0);
		}
		else if(stick->GetRawButton(3))
		{
			gathererLifter->Set(ControlMode::PercentOutput, -1.0);
		}
		else
		{
			gathererLifter->Set(ControlMode::PercentOutput, 0.0);
		}
	}

	void TestPeriodic() {}

private:
};

START_ROBOT_CLASS(Robot)
