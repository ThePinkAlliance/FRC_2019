/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.utils.PresetPositions;

public class MoveBallRocket extends Command {
  public MoveBallRocket() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_ball);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_ball.setRotateMotorCmd(PresetPositions.BALL_LOW_ROCKET_POSITION);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return Math.abs(Robot.m_ball.getBallRotateEncoder() - PresetPositions.BALL_LOW_ROCKET_POSITION) <= 100;
    return Math.abs(Robot.m_oi.getTowerJoystick().getRawAxis(1)) > 0.05;
  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Robot.m_ball.holdRotation();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
