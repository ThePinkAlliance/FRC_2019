package frc.robot;


import edu.wpi.first.wpilibj.buttons.Button;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.MotionProfileLevel2GroupClimb;
import frc.robot.commands.MotionProfileLevel3GroupClimb;
import frc.robot.commands.MotionProfileGroupClimb;
import frc.robot.commands.MoveBallRocket;
import frc.robot.commands.MoveBallToStow;
import frc.robot.commands.MoveElevatorToCollect;
import frc.robot.commands.MoveElevatorToMax;
import frc.robot.commands.MoveElevatorToMidRocket;
import frc.robot.commands.OpenBeak;
import frc.robot.commands.StartupCollectHatch;
import frc.robot.commands.StartupCollectHatchGroup;
import frc.robot.commands.ToggleNeck;
import frc.robot.commands.AutomatedCollect;
import frc.robot.commands.CloseBeak;
import frc.robot.commands.Collect;
import frc.robot.commands.Eject;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //============================================
	// 		JOYSTICK VARIABLES:  XBOX CONTROLLER
	//============================================
  public static int baseJoystickPort = 0;
  public static int towerJoystickPort = 1;
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
  //Axis values
  public static int LXAxis = 0;
  public static int leftStick = 1; //aka LYAxis
  public static int RXAxis = 2;
  public static int rightStick = 3; //aka RYAxis

  //public Joystick base = new Joystick(baseJoystickPort);
  public Joystick base = null; 
  public Joystick tower = null;
  public Button testButton = null; 
  public Button baseB = null;
  public Button baseY = null;
  public Button baseA = null;
  public Button baseX = null;
  public Button towerX = null;
  public Button towerA = null;
  public Button towerY = null;
  public Button towerB = null;
  public Button baseStart = null;
  public Button baseSelect = null;
  public Button towerRightBumper = null;
  public Button baseRightTrigger = null;
  public Button baseLeftTrigger = null;
  public Button towerRightTrigger = null;
  public Button baseRightBumper = null;
  public Button towerStart = null;
  public Button towerSelect = null;
	
	public OI() {

    try {

      // setup your joystick
      base = new Joystick(baseJoystickPort);
      tower = new Joystick(towerJoystickPort);
      baseA = new JoystickButton(base, aButtonNumber);
      baseX = new JoystickButton(base, xButtonNumber);
      baseB = new JoystickButton(base, bButtonNumber);
      baseY = new JoystickButton(base, yButtonNumber);
      towerX = new JoystickButton(tower, xButtonNumber);
      towerA = new JoystickButton(tower, aButtonNumber);
      towerY = new JoystickButton(tower, yButtonNumber);
      towerB = new JoystickButton(tower, bButtonNumber);
      baseStart = new JoystickButton(base, startButtonNumber);
      baseSelect = new JoystickButton(base, selectButtonNumber);
      towerRightBumper = new JoystickButton(tower, rightBumperButtonNumber);
      baseRightTrigger = new JoystickButton(base, rightTriggerButtonNumber);
      baseLeftTrigger = new JoystickButton(base, leftTriggerButtonNumber);
      towerStart = new JoystickButton(tower, startButtonNumber);
      towerSelect = new JoystickButton(tower, selectButtonNumber);
      baseRightBumper = new JoystickButton(base, rightBumperButtonNumber);

      //Enable buttons / actions 
      setupBaseJoystick();
      setupTowerJoystick();

    } catch (Exception e) {
      //System..out.println("Error setting up joystick.");
      System.out.println(e.toString());
    }

  }
  
  public void setupBaseJoystick() {
    if (base != null) {
      baseY.whenPressed(new ToggleNeck());
      baseB.toggleWhenPressed(new StartupCollectHatchGroup());
      baseX.whenPressed(new OpenBeak());
      baseA.whenPressed(new CloseBeak());
      baseRightTrigger.toggleWhenPressed(new AutomatedCollect());
      baseLeftTrigger.whileHeld(new Eject());
      baseRightBumper.toggleWhenPressed(new Collect());
      baseStart.whenPressed(new MotionProfileLevel3GroupClimb(false)); // Automated Climb to Level 3
      baseSelect.whenPressed(new MotionProfileLevel2GroupClimb(false)); // Automated Climb to Level 2
    }
  }

  public void setupTowerJoystick() {
    if (tower != null) {
      towerA.whenPressed(new MoveElevatorToCollect());
      towerX.whenPressed(new MoveElevatorToMidRocket());
      towerB.whenPressed(new MoveBallRocket());
      towerY.whenPressed(new MoveElevatorToMax());
      towerRightBumper.whenPressed(new MoveBallToStow());
      towerStart.whenPressed(new MotionProfileGroupClimb());  // Manual Climb to Level 3
      towerSelect.whenPressed(new MotionProfileLevel2GroupClimb(true)); // Manual Climb to  Level 2

    }
  }
	
	public Joystick getBaseJoystick() {
		return base;
  }
  
  public Joystick getTowerJoystick() {
    return tower;
  }
}