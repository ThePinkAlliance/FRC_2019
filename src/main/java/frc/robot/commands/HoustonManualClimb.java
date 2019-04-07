/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.HoustonClimber;
import frc.robot.subsystems.HoustonClimber.ClimbLevel;
import frc.robot.subsystems.HoustonClimber.PodAction;
import frc.robot.subsystems.utils.HoustonMotionProfileExecutor;
import frc.robot.Robot;



public class HoustonManualClimb extends Command {

  private HoustonMotionProfileExecutor mp = null;
  private PodAction action = PodAction.CLIMB;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private HoustonClimber climberPod = null;
  private ClimbLevel level = ClimbLevel.LEVEL3;
  
  public HoustonManualClimb(PodAction action, ClimbLevel level, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    this.climberPod = Robot.m_climber;

    //set the direction
    this.action = action;

    //set the level
    this.level = level;

    //cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;

    //new up the timer for later use
    watchDog = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
        //prep your time
        watchDog.reset();
        watchDog.start();
     
        //get the motion profile object associated with the subsystem
        mp = climberPod.getMP();
        climberPod.setAction(action);
        mp.reset();
        climberPod.resetEncoderPosition(0);
        mp.setMotionProfileMode();
        mp.startMotionProfile();
        System.out.println("HoustonMotionProfileExecutor(): initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      mp.control(action, level);
      mp.setMotionProfileMode();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double elapsedTime = watchDog.get();
    boolean bMPDone = mp.isMotionProfileDone();
    
    if (elapsedTime >= watchDogTime) {
      System.out.println("Watch Dog timer popped.");
      return true;
    }

    return bMPDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mp.stopMotionProfile();
    System.out.println("Houston Climb: End");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mp.stopMotionProfile();
    System.out.println("Houston Climb: Interrupted");
  }
}
