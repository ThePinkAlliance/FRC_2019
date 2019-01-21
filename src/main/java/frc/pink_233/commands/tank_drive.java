/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.pink_233.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.pink_233.oi;
import frc.pink_233.robot;
import frc.pink_233.robot_dashboard;;

public class tank_drive extends Command {

  private Joystick js = null; 
  
  /**
   * Constructor: needs require and sets the member js so that it can be used
   * through out the instance of the object.
   */
  public tank_drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(robot.drive_train);
    js = robot.m_oi.getBaseJoystick();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  /** 
   * Perform tankdrive action during execute() calls
   */
  public void HandleTankDrive() {
    if (js != null) {
      double left = js.getRawAxis(oi.leftStick);//js.getY(Hand.kLeft);
      double right =  js.getRawAxis(oi.rightStick);//js.getY(Hand.kRight);
      //System.out.println("LEFT: " + left + " RIGHT: " + right);
      robot.drive_train.tankDriveByJoystick(left, right);
    }
  }
  
  /**
   * Perform any button actions during execute calls
   */
  public void HandleButtons() {
    if (js != null) {
      //Reset Encoders
      if (js.getRawButtonPressed(oi.xButtonNumber)) {
        robot.drive_train.resetEncoders();
      }

      if (js.getRawButtonPressed(oi.aButtonNumber)) {
        encoder_based_drive_pid_control testCmd = new encoder_based_drive_pid_control(
          SmartDashboard.getNumber(robot_dashboard.DT_ENC_PID_DISTANCE, 30), 
          10, 
          SmartDashboard.getNumber(robot_dashboard.DT_ENC_PID_MAX_OUTPUT, 1));
        testCmd.start();
      }
    }
  }
}