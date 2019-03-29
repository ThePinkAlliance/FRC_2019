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
import frc.robot.subsystems.utils.MotionProfileClimberDouble;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimbLevel;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;
import frc.robot.Robot;



public class MotionProfileClimberManual3 extends Command {

  private MotionProfileClimberDouble mp = null;
  private ClimberDirection direction = ClimberDirection.UP;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private MotionProfileClimber climberPod = null;
  private PodPosition location;

  private ClimbLevel level = ClimbLevel.LEVEL3;

  private Timer profileStartTimer = null;

  public MotionProfileClimberManual3(MotionProfileClimber theClimberPod, ClimberDirection direction, PodPosition location, double preLoadMove, double watchDogTime, double profileDelayTime) {
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

    double climberPosition = 0.0; 

    if (climberPod.isMotionProfileFinished()) {
      climberPod.setPosition(climberPosition);
      //climberPod.set(0.25);
    } else {
      mp.control(direction, location, level);
      mp.setMotionProfileMode();
      climberPosition = climberPod.getEncPosition();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //boolean mpPressed = Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    //System.out.println("mpPressed: " + mpPressed);
    double elapsedTime = watchDog.get();
    boolean bMPDone = mp.isMotionProfileDone();
    boolean bMPOtherSideDone = true;
    if (climberPod.getSide() == PodPosition.RIGHT) {
      bMPOtherSideDone = Robot.m_climberPodFrontLeft.isMotionProfileFinished();
    } else {
      bMPOtherSideDone = Robot.m_climberPodFrontRight.isMotionProfileFinished();
    }
    bMPDone = (bMPDone && bMPOtherSideDone);
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
