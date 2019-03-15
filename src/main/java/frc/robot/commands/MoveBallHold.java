/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveBallHold extends Command {

  private double positionToHold = 0;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;

  public MoveBallHold(double commandedtime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_ball);
    watchDogTime = commandedtime;

    watchDog = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    watchDog.reset();
    watchDog.start();
    positionToHold = Robot.m_ball.getBallRotateEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_ball._collectorRotateMotor.set(ControlMode.Position, positionToHold);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double elapsedTime = watchDog.get();

    if (elapsedTime >= watchDogTime) {
      return true;
    }
    
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
  }
}
