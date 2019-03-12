/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveClimberWheels extends Command {

  private Timer watchDogTimer = null;
  private Timer cruiseTimer = null;
  private double watchDogTime = 10.0;
  private double cruiseTime = 4.0;
  private double motorPower = 0.0;
  private double rampRate = .02;
  private double maxPower = 1;

  private boolean ramped = false;
  private boolean cruiseStarted = false;
  private boolean isDone = false;


  public DriveClimberWheels() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climberDriver);

    watchDogTimer = new Timer();
    cruiseTimer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    watchDogTimer.reset();
    watchDogTimer.start();


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (!ramped && motorPower < maxPower) {
      motorPower += rampRate;
      Robot.m_climberDriver.setClimberWheels(motorPower);
      return;
    }

    ramped = true;

    if (!cruiseStarted) {
     cruiseTimer.reset();
     cruiseTimer.start();
     cruiseStarted = true;
    }

    if(cruiseTimer.get() < cruiseTime) {
      return;
    }

    if (motorPower > 0) {
      motorPower -= rampRate;
      Robot.m_climberDriver.setClimberWheels(motorPower);
    }
    else {
      isDone = true;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    double elapsedTime = watchDogTimer.get();

    if (elapsedTime >= watchDogTime || isDone) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climberDriver.setClimberWheels(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_climberDriver.setClimberWheels(0);
  }
}
