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
import frc.robot.subsystems.MotionProfileClimberMaster;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;


public class HoldClimberPosition extends Command {

  private MotionProfileClimberMaster climberPod = null;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;


  public HoldClimberPosition(double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climberMaster);

    this.climberPod = Robot.m_climberMaster;
    this.watchDogTime = watchDogTime;

    watchDog = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    //start the timer
    watchDog.reset();
    watchDog.start();

    // positionToHoldFR = climberPod.getEncPosition(PodPosition.FRONT, PodPosition.RIGHT);
    // positionToHoldFL = climberPod.getEncPosition(PodPosition.FRONT, PodPosition.LEFT);
    // positionToHoldBR = climberPod.getEncPosition(PodPosition.BACK, PodPosition.RIGHT);
    // positionToHoldBL = climberPod.getEncPosition(PodPosition.BACK, PodPosition.LEFT);


    // climberPod.setPosition(PodPosition.FRONT, PodPosition.RIGHT, positionToHoldFR);
    // climberPod.setPosition(PodPosition.FRONT, PodPosition.LEFT, positionToHoldFL);
    // climberPod.setPosition(PodPosition.BACK, PodPosition.RIGHT, positionToHoldBR);
    // climberPod.setPosition(PodPosition.BACK, PodPosition.LEFT, positionToHoldBL);

    climberPod.set(PodPosition.FRONT, PodPosition.RIGHT, 0.1);
    climberPod.set(PodPosition.FRONT, PodPosition.LEFT, 0.1);
    climberPod.set(PodPosition.BACK, PodPosition.RIGHT, 0.15);
    climberPod.set(PodPosition.BACK, PodPosition.LEFT, 0.15);

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
    climberPod.set(PodPosition.FRONT, PodPosition.RIGHT, 0);
    climberPod.set(PodPosition.FRONT, PodPosition.LEFT, 0);
    climberPod.set(PodPosition.BACK, PodPosition.RIGHT, 0);
    climberPod.set(PodPosition.BACK, PodPosition.LEFT, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

    //set climber pod motor to zero
    climberPod.set(PodPosition.FRONT, PodPosition.RIGHT, 0);
    climberPod.set(PodPosition.FRONT, PodPosition.LEFT, 0);
    climberPod.set(PodPosition.BACK, PodPosition.RIGHT, 0);
    climberPod.set(PodPosition.BACK, PodPosition.LEFT, 0);
  }
}
