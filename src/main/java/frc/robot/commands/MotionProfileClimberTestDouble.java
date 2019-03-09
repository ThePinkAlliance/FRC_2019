/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.MotionProfileClimber;
import frc.robot.subsystems.MotionProfileClimber.ClimberDirection;
import frc.robot.subsystems.MotionProfileClimber.PodPosition;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;

public class MotionProfileClimberTestDouble extends Command {

  private Joystick js = null; 
  private MotionProfileClimberDouble mp = null;
  private ClimberDirection direction = ClimberDirection.UP;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private final double UNWIND_TIME = 0.0;  //one sec to let talon unwind
  private double doneTime = 0;
  private MotionProfileClimber climberPod = null;
  private PodPosition location;
  
  /**
   * 
   * @param direction which direction are we going UP or DOWN.  This affects
   *                  which motion profile is loaded
   * @param watchDogTime amount of time this command must complete in
   * 
   */
  public MotionProfileClimberTestDouble(MotionProfileClimber theClimberPod, ClimberDirection direction, PodPosition location, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);
    this.climberPod = theClimberPod;

    //set the direction
    this.direction = direction;

    // set the position
    this.location = location;
   
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

    if (direction == ClimberDirection.UP) {
      // go to 0 if we are going up
      
    } else {
      // go to max if we are going down
    }

    //get the motion profile object associated with the subsystem
    mp = climberPod.getMP();
    climberPod.setDirection(direction);
    mp.reset();
    climberPod.resetEncoderPosition(0);
    mp.setMotionProfileMode();
    //mp.startWorking(movingUp); //only used by threading alternative
    mp.startMotionProfile();
    System.out.println("MotionProfileTestClimberDouble(): initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mp.control(direction, location);
    mp.setMotionProfileMode();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //boolean mpPressed = Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    //System.out.println("mpPressed: " + mpPressed);
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
    //mp.stopWorking();
    System.out.println("MotionProfileTestClimberDouble(): End");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mp.stopMotionProfile();
   // mp.stopWorking();  //only used by threading alternative
   System.out.println("MotionProfileTestClimberDouble(): Interrupted");
  }

}