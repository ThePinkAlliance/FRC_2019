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
import frc.robot.subsystems.MotionProfileClimber;


public class MotionProfileClimberHoldDefault extends Command {

  private MotionProfileClimber climberPod = null;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;

  private double positionToHold = 0.0;
  // private double positionToHoldFL = 0.0;
  // private double positionToHoldBR = 0.0;
  // private double positionToHoldBL = 0.0;
  

  public MotionProfileClimberHoldDefault(MotionProfileClimber theClimberPod) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);

    this.climberPod = theClimberPod;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    //start the timer
    watchDog.reset();
    watchDog.start();

    positionToHold = 0;
    // positionToHoldFL = climberPod.getEncPosition(PodPosition.FRONT, PodPosition.LEFT);
    // positionToHoldBR = climberPod.getEncPosition(PodPosition.BACK, PodPosition.RIGHT);
    // positionToHoldBL = climberPod.getEncPosition(PodPosition.BACK, PodPosition.LEFT);


    climberPod.setPosition(positionToHold);
    // climberPod.setPosition(PodPosition.FRONT, PodPosition.LEFT, positionToHoldFL);
    // climberPod.setPosition(PodPosition.BACK, PodPosition.RIGHT, positionToHoldBR);
    // climberPod.setPosition(PodPosition.BACK, PodPosition.LEFT, positionToHoldBL);

    // climberPod.set(PodPosition.FRONT, PodPosition.RIGHT, 0.1);
    // climberPod.set(PodPosition.FRONT, PodPosition.LEFT, 0.1);
    // climberPod.set(PodPosition.BACK, PodPosition.RIGHT, 0.15);
    // climberPod.set(PodPosition.BACK, PodPosition.LEFT, 0.15);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    climberPod.setPosition(positionToHold);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
 
    //set climber pod motor to zero
    climberPod.set(0);
    // climberPod.set(PodPosition.FRONT, PodPosition.RIGHT, 0);
    // climberPod.set(PodPosition.FRONT, PodPosition.LEFT, 0);
    // climberPod.set(PodPosition.BACK, PodPosition.RIGHT, 0);
    // climberPod.set(PodPosition.BACK, PodPosition.LEFT, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

    //set climber pod motor to zero
    climberPod.set(0);
    // climberPod.set(PodPosition.FRONT, PodPosition.RIGHT, 0);
    // climberPod.set(PodPosition.FRONT, PodPosition.LEFT, 0);
    // climberPod.set(PodPosition.BACK, PodPosition.RIGHT, 0);
    // climberPod.set(PodPosition.BACK, PodPosition.LEFT, 0);
  }
}