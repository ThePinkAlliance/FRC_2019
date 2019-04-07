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
import frc.robot.subsystems.HoustonClimber.PodAction;
import frc.robot.Robot;



public class HoustonClimberPrep extends Command {
  private PodAction action = PodAction.CLIMB;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private HoustonClimber climberPod = null;
  private double prepPower = 0.0;
  
  public HoustonClimberPrep(PodAction action, double prepPower, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    this.climberPod = Robot.m_climber;

    //set the direction
    this.action = action;

    //set the prep power
    prepPower = Math.abs(prepPower);
    this.prepPower = ((this.action == PodAction.CLIMB) ? prepPower : -prepPower);

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
        //By having the default invert, we can safely decide on which sign touse on prepPower
        climberPod.setupLeftDefault();
        climberPod.setupRightDefault();
        System.out.println("Prep: initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     climberPod.set(prepPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double elapsedTime = watchDog.get();
    
    
    if (elapsedTime >= watchDogTime) {
      System.out.println("Prep: Watch Dog timer popped.");
      return true;
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climberPod.set(0);
    System.out.println("Prep: End");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    climberPod.set(0);
    System.out.println("Prep: Interrupted");
  }
}
