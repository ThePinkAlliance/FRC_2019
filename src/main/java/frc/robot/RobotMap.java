package frc.robot;

// The RobotMap is a mapping from the ports sensors and
// actuators are wired into to a variable name.
public class RobotMap {
	// Motors
	public static int rightBackClimberPort = 22; //From collector POV
	public static int leftBackClimberPort = 21;  //From collector POV
	public static int climberSmartsForwardRightMotor = 20;  //From collector POV
	public static int climberSmartsForwardLeftMotor = 23; // Added Motor
	public static int rightFrontMotorPort = 11;
	public static int rightBackMotorPort = 13;  
	public static int leftFrontMotorPort = 2;
	public static int leftBackMotorPort = 3;
	public static int climberMotorPort = 99;
	public static int elevatorMotorPort = 5;
	public static int collectorMotorPort = 1;
	public static int collectorRotateMotorPort = 4;

	// Pnuematics
	public static int beakPort = 0;
	public static int neckPort = 1;

	// Switches
	public static int limitSwitchHatchCollectedPort = 0;
	public static int elevatorBottomLimitSwitchPort = 1;
	public static int elevatorTopLimitSwitchPort = 2;
	public static int collectedOpticalSwitchPort = 3;
	public static int climberBackLeftTopSwitchPort = 4;
	public static int climberBackLeftBottomSwitchPort = 5;
	public static int climberBackRightTopSwitchPort = 6;
	public static int climberBackRightBottoSwitchmPort = 7;
	public static int bellyPanFrontSwitchPort = 8;
	public static int bellyPanRearSwitchPort = 9;

}