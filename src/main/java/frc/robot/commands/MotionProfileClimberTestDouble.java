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

  private Timer profileStartTimer = null;
  private double delayTime = 0;
  private double moveVoltage = 0;
  private boolean motionProfileStarted = false;
  
  /**
   * 
   * @param direction which direction are we going UP or DOWN.  This affects
   *                  which motion profile is loaded
   * @param watchDogTime amount of time this command must complete in
   * 
   */
  public MotionProfileClimberTestDouble(MotionProfileClimber theClimberPod, ClimberDirection direction, PodPosition location, double preLoadMove, double watchDogTime, double profileDelayTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);
    this.climberPod = theClimberPod;

    //set the direction
    this.direction = direction;

    // set the position
    this.location = location;

    //voltage to send to the motors before running profile
    this.moveVoltage = preLoadMove;
   
    //cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;

    //how long to move motors before starting profile
    this.delayTime = profileDelayTime;

    //new up the timer for later use
    watchDog = new Timer();
    profileStartTimer = new Timer();
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

    //start the timer to delay the command
    profileStartTimer.reset();
    profileStartTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // //get time elapsed
    // double delayElapsedTime = profileStartTimer.get();

    // //if timer popped and profile not started
    // if (delayElapsedTime >= delayTime && !motionProfileStarted)  {
    //   //set flag and start motion profile
    //   climberPod.set(0);
    //   climberPod.resetEncoderPosition(0);

    //   motionProfileStarted = true;
    //   mp.startMotionProfile();
    // }
    // else if (delayElapsedTime >= delayTime && motionProfileStarted) {
    //   //continue executing motion profile
       mp.control(direction, location);
       mp.setMotionProfileMode();
    // }
    // else {
    //   //otherwise set voltage to pod
    //   this.climberPod.set(moveVoltage);
    // }
   
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