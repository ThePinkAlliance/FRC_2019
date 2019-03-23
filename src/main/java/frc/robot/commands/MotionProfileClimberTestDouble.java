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
import frc.robot.Robot;
import frc.robot.subsystems.MotionProfileClimber;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimbLevel;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;

public class MotionProfileClimberTestDouble extends Command {

  private MotionProfileClimberDouble mp = null;
  private ClimberDirection direction = ClimberDirection.UP;
  private ClimbLevel level = ClimbLevel.LEVEL3;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private MotionProfileClimber climberPod = null;
  private PodPosition location;
  private double p_gain;
  private double target_position;
  private boolean manual_override = false;

  private Timer profileStartTimer = null;

  /**
   * 
   * @param direction    which direction are we going UP or DOWN. This affects
   *                     which motion profile is loaded
   * @param watchDogTime amount of time this command must complete in
   * 
   */
  public MotionProfileClimberTestDouble(MotionProfileClimber theClimberPod, ClimberDirection direction,
      PodPosition location, ClimbLevel level, double preLoadMove, double watchDogTime, double proportionalGain,
      double position, boolean manualOverride) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);
    if (theClimberPod.getSide() == PodPosition.LEFT) {
      requires(Robot.m_ball);
    }

    this.climberPod = theClimberPod;

    // set the direction
    this.direction = direction;

    // set the position
    this.location = location;

    // set the climb level
    this.level = level;

    // cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;

    manual_override = manualOverride;
    p_gain = proportionalGain;
    target_position = position;

    // new up the timer for later use
    watchDog = new Timer();
    profileStartTimer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // prep your time
    watchDog.reset();
    watchDog.start();

    if (direction == ClimberDirection.UP) {
      // go to 0 if we are going up

    } else {
      // go to max if we are going down
    }

    // get the motion profile object associated with the subsystem
    mp = climberPod.getMP();
    climberPod.setDirection(direction);
    mp.reset();
    climberPod.resetEncoderPosition(0);
    mp.setMotionProfileMode();
    // mp.startWorking(movingUp); //only used by threading alternative
    mp.startMotionProfile();
    System.out.println("MotionProfileTestClimberDouble(): initialized");

    // start the timer to delay the command
    profileStartTimer.reset();
    profileStartTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // //get time elapsed
    // double delayElapsedTime = profileStartTimer.get();

    // //if timer popped and profile not started
    // if (delayElapsedTime >= delayTime && !motionProfileStarted) {
    // //set flag and start motion profile
    // climberPod.set(0);
    // climberPod.resetEncoderPosition(0);

    // motionProfileStarted = true;
    // mp.startMotionProfile();
    // }
    // else if (delayElapsedTime >= delayTime && motionProfileStarted) {
    // //continue executing motion profile
    if (manual_override) {
      mp.control(direction, location, level);
      mp.setMotionProfileMode();
    } else {
      mp.control(direction, location, level);
      mp.setMotionProfileMode();
      if (climberPod.getSide() == PodPosition.LEFT) {
        Robot.m_ball.setClimberRotateMotorCmd(target_position, p_gain);
      }
    }
    // }
    // else {
    // //otherwise set voltage to pod
    // this.climberPod.set(moveVoltage);
    // }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // boolean mpPressed =
    // Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    // System.out.println("mpPressed: " + mpPressed);
    double elapsedTime = watchDog.get();
    boolean bMPDone = false;
    if (direction == ClimberDirection.UP || manual_override) {
      bMPDone = mp.isMotionProfileDone();
    } else {
      bMPDone = (mp.isMotionProfileDone()
          && (Math.abs(Robot.m_ball.getBallRotateEncoder() - Robot.m_ball.CARGO_POS) <= 100));
    }
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
    if (climberPod.getSide() == PodPosition.LEFT && !manual_override) {
      Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput,0);
    }
    // mp.stopWorking();
    System.out.println("MotionProfileTestClimberDouble(): End");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mp.stopMotionProfile();
    if (climberPod.getSide() == PodPosition.LEFT && !manual_override) {
      Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput,0);
    }
    // mp.stopWorking(); //only used by threading alternative
    System.out.println("MotionProfileTestClimberDouble(): Interrupted");
  }

}