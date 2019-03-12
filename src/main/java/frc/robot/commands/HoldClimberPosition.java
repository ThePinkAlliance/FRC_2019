/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.MotionProfileClimber;


public class HoldClimberPosition extends Command {

  private MotionProfileClimber climberPod = null;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;

  private double positionToHold = 0.0;
  

  public HoldClimberPosition(MotionProfileClimber theClimberPod, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);

    this.climberPod = theClimberPod;
    this.watchDogTime = watchDogTime;

    watchDog = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    //start the timer
    watchDog.reset();
    watchDog.start();

    positionToHold = climberPod.getEncPosition();

    climberPod.setPosition(positionToHold);
  

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
    climberPod.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

    //set climber pod motor to zero
    climberPod.set(0);
  }
}
