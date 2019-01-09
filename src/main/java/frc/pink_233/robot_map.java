package frc.pink_233;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class robot_map {
	
	//============================================
	// 		DRIVE TRAIN VARIABLES
	//============================================
	// Motors
	public static int leftFrontMotorPort = 2;
	public static int leftBackMotorPort = 3;
	public static int rightFrontMotorPort = 0;
	public static int rightBackMotorPort = 1;
	
	// Encoders
	public static int leftEncoderAPort = 0;
	public static int leftEncoderBPort = 1;
	public static int rightEncoderAPort = 2;
	public static int rightEncoderBPort = 3;

	

	
	//============================================
	// 		JOYSTICK VARIABLES
	//============================================
	public static int baseJoystickPort = 0;
	
	// Define all raw button numbers
	public static int xButtonNumber = 1;
	public static int aButtonNumber = 2;
	public static int bButtonNumber = 3;
	public static int yButtonNumber = 4;
	public static int leftBumperButtonNumber = 5;
	public static int rightBumperButtonNumber = 6;
	public static int leftTriggerButtonNumber = 7;
	public static int rightTriggerButtonNumber = 8;
	public static int selectButtonNumber = 9;
	public static int startButtonNumber = 10;
	public static int leftJoystickButtonNumber = 11;
	public static int rightJoystickButtonNumber = 12;
	
}