/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class Collect extends Command {

  // Init sticks
  private Joystick js = null;
  private double stickValue = 0;

  public Collect() {
    
    // Requires Ball collector
    requires(Robot.m_ball);
    js = new Joystick(OI.towerJoystickPort);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // If collectedOpticalSwitch, collect ball
    if (Robot.m_ball.collectedOpticalSwitch.get()) {
      Robot.m_ball.collect();
      //system..out.println("Switch: True");
      if (js.getRawAxis(OI.leftStick) > 0.1 || js.getRawAxis(OI.leftStick) < -0.1) {
        ReadJoystick();
      } else if (js.getPOV() == 0) {
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.CARGO_POS);
      } else if (js.getPOV() == 90) {
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.LOW_ROCKET_POS);
      } else if (js.getPOV() == 180) {
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.COLLECT_POS);
      } else {
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.getBallRotateEncoder());
      }
    } 
    // Else, hold ball that has been collected
    else {
      Robot.m_ball.hold();
      //system..out.println("Switch: False");
      if (js.getRawAxis(OI.leftStick) > 0.1 || js.getRawAxis(OI.leftStick) < -0.1) {
        ReadJoystick();
      } else if (js.getPOV() == 0) {
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.CARGO_POS);
      } else if (js.getPOV() == 90) {
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.LOW_ROCKET_POS);
      } else if (js.getPOV() == 180) {
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.COLLECT_POS);
      } else {
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.getBallRotateEncoder());
      }
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_ball.hold();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_ball.hold();
  }

  // Method to set motor power based of the stickValue
  public void ReadJoystick() {
    // Read out stickValue
        stickValue = js.getRawAxis(OI.leftStick);
      // Set _elevator Motor to stickValue
      Robot.m_ball.moveBall(stickValue);
    }
  }