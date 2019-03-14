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



public class MotionProfileClimberDriveTrain extends Command {

  
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private double speed = 0.0;
  

  public MotionProfileClimberDriveTrain(double speed, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);

    this.watchDogTime = watchDogTime;
    this.speed = speed;
    watchDog = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    //start the timer
    watchDog.reset();
    watchDog.start();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_driveTrain.tankDrive(speed, speed);
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
    //set climber pod motor to zero
    Robot.m_driveTrain.tankDrive(0,0); 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.tankDrive(0,0);
  }
}
