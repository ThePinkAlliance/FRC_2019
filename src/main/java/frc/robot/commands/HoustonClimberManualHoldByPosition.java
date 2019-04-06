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
import frc.robot.subsystems.HoustonClimber;


public class HoustonClimberManualHoldByPosition extends Command {

  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private HoustonClimber climberPod = null;
  private double positionToHold = 0.0;

  public HoustonClimberManualHoldByPosition(double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    this.climberPod = Robot.m_climber;
    //cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;
    
    //new up the timer for later use
    watchDog = new Timer();
 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // prep your time
    watchDog.reset();
    watchDog.start();
    positionToHold = climberPod.getEncPositionMaster();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    climberPod.setPosition(positionToHold);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //boolean mpPressed = Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    //System.out.println("mpPressed: " + mpPressed);
    double elapsedTime = watchDog.get();
    if (elapsedTime >= watchDogTime) {
      System.out.println("Watch Dog timer popped.");
      return true;
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climberPod.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    climberPod.set(0);
  }
}
