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
import frc.robot.subsystems.MotionProfileBall;
import frc.robot.subsystems.utils.MotionProfileBallDouble;
import frc.robot.subsystems.utils.MotionProfileBallDouble.CollectorDirection;


public class MotionProfileBallClimbSequence extends Command {

  private MotionProfileBallDouble mp = null;
  private CollectorDirection direction = CollectorDirection.UP;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private MotionProfileBall collector = null;
  

  private Timer profileStartTimer = null;
  
  /**
   * 
   * @param direction    which direction are we going UP or DOWN. This affects
   *                     which motion profile is loaded
   * @param watchDogTime amount of time this command must complete in
   * 
   */
  public MotionProfileBallClimbSequence(CollectorDirection direction, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_ball);
    collector = Robot.m_ball;
    //set the direction
    this.direction = direction;

    //cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;

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

    

    //get the motion profile object associated with the subsystem
    mp = collector.getMP();
    collector.setDirection(direction);
    mp.reset();
    collector.resetEncoderPosition(0);
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
    //   collector.set(0);
    //   collector.resetEncoderPosition(0);

    //   motionProfileStarted = true;
    //   mp.startMotionProfile();
    // }
    // else if (delayElapsedTime >= delayTime && motionProfileStarted) {
    //   //continue executing motion profile
       mp.control(direction);
       mp.setMotionProfileMode();
    // }
    // else {
    //   //otherwise set voltage to pod
    //   this.collector.set(moveVoltage);
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