/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.OI;
import frc.robot.Robot;

public class JoystickDrive extends Command {

  private Joystick js = null; 
  
  /**
   * Constructor: needs require and sets the member js so that it can be used
   * through out the instance of the object.
   */
  public JoystickDrive() {
    
    // Requires Drive Train
    requires(Robot.m_driveTrain);
    js = Robot.m_oi.getBaseJoystick();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    HandleButtons();
    HandleTankDrive();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.leftMotorStop();
    Robot.m_driveTrain.rightMotorStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.leftMotorStop();
    Robot.m_driveTrain.rightMotorStop();
  }

  /** 
   * Perform tankdrive action during execute() calls
   */
  public void HandleTankDrive() {
    if (js != null) {
      double left = js.getRawAxis(OI.leftStick);//js.getY(Hand.kLeft);
      double right =  js.getRawAxis(OI.rightStick);//js.getY(Hand.kRight);
      System.out.println("LEFT: " + left + " RIGHT: " + right);
      Robot.m_driveTrain.tankDriveByJoystick(left, right);
    }
  }
  
  /**
   * Perform any button actions during execute calls
   */
  public void HandleButtons() {
    if (js != null) {

      //Reset Gyro
      if (js.getRawButtonPressed(OI.bButtonNumber)) {
        Robot.m_driveTrain.resetGyro();
      }
    }
  }
}
