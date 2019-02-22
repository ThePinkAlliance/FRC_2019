/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.buttons.Button;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Hold;
import frc.robot.commands.OpenBeak;
import frc.robot.commands.ToggleNeck;
import frc.robot.commands.CloseBeak;
import frc.robot.commands.Collect;
import frc.robot.commands.Eject;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

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
  public Button baseRightBumper = null;
  public Button baseLeftBumper = null;
  public Button towerRightTrigger = null;
	
	public OI() {

    try {

      //Setup your joystick
      base = new Joystick(baseJoystickPort);
      tower = new Joystick(towerJoystickPort);
      baseA = new JoystickButton(base, aButtonNumber);
      baseX = new JoystickButton(base, xButtonNumber);
      //baseB = new JoystickButton(base, bButtonNumber);
      baseY = new JoystickButton(base, yButtonNumber);
      baseRightBumper = new JoystickButton(base, rightBumperButtonNumber);
      baseLeftBumper = new JoystickButton(base, leftBumperButtonNumber);

      //Enable buttons / actions 
      setupBaseJoystick();
      setupTowerJoystick();

    } catch (Exception e) {
      System.out.println("Error setting up joystick.");
      System.out.println(e.toString());
    }

  }
  
  public void setupBaseJoystick() {
    if (base != null) {
       baseY.whenPressed(new ToggleNeck());
       //baseB.whenPressed(new ToggleBeak());
       baseA.whenPressed(new OpenBeak());
       baseX.whenPressed(new CloseBeak());
       
       baseRightBumper.whenPressed(new Collect());
       baseRightBumper.whenReleased(new Hold());

       baseLeftBumper.whenPressed(new Eject());
       baseLeftBumper.whenReleased(new Hold());
    }
  }

  public void setupTowerJoystick() {
    if (tower != null) {
    }
  }
	
	public Joystick getBaseJoystick() {
		return base;
  }
  
  public Joystick getTowerJoystick() {
    return tower;
  }
}