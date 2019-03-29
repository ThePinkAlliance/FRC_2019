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
import frc.robot.subsystems.utils.MotionProfileBallDouble;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;
import frc.robot.subsystems.utils.MotionProfileBallDouble.CollectorDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimbLevel;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;

public class MotionProfileClimberTestDoubleLevel3 extends Command {

  private MotionProfileClimberDouble climber_mp = null;
  private MotionProfileBallDouble ball_mp = null;
  private ClimberDirection direction = ClimberDirection.UP;
  private CollectorDirection ball_direction = CollectorDirection.UP;
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
  public MotionProfileClimberTestDoubleLevel3(MotionProfileClimber theClimberPod, ClimberDirection direction,
      PodPosition location, ClimbLevel level, double preLoadMove, double watchDogTime, double proportionalGain,
      double position, boolean manualOverride, CollectorDirection ball_direction) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);
    if (theClimberPod.getSide() == PodPosition.LEFT && !manualOverride) {
      requires(Robot.m_ball);
    }

    this.climberPod = theClimberPod;

    this.ball_direction = ball_direction;
    
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
    climber_mp = climberPod.getMP();
    climberPod.setDirection(direction);
    climber_mp.reset();
    climberPod.resetEncoderPosition(0);
    climber_mp.setMotionProfileMode();
    // mp.startWorking(movingUp); //only used by threading alternative
    climber_mp.startMotionProfile();
    System.out.println("MotionProfileTestClimberDouble(): initialized");

    // get the motion profile object associated with the subsystem
    if (climberPod.getSide() == PodPosition.LEFT && !manual_override) {
      if (ball_direction == CollectorDirection.UP) {
        ball_mp = Robot.m_ball.getMP();
        Robot.m_ball.setDirection(ball_direction);
        ball_mp.reset();
        Robot.m_ball.resetEncoderPosition(0);
        ball_mp.setMotionProfileMode();
        ball_mp.startMotionProfile();
        System.out.println("MotionProfileTestClimberDouble(): initialized");
      } else {
        Robot.m_ball._collectorRotateMotor.set(ControlMode.Position, target_position);
      }
    }

    // start the timer to delay the command
    profileStartTimer.reset();
    profileStartTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double climberPosition = 0.0; 
    double ballPosition = 0.0;
    if (manual_override) {
      climber_mp.control(direction, location, level);
      climber_mp.setMotionProfileMode();
    } else {
      if (climberPod.isMotionProfileFinished()) {
        climberPod.setPosition(climberPosition);
        //climberPod.set(0.25);
      } else {
        climber_mp.control(direction, location, level);
        climber_mp.setMotionProfileMode();
        climberPosition = climberPod.getEncPosition();
      }

      if (climberPod.getSide() == PodPosition.LEFT) {
        if (ball_direction == CollectorDirection.UP) {
          if (Robot.m_ball.isMotionProfileFinished()) {
            Robot.m_ball._collectorRotateMotor.set(ControlMode.Position, ballPosition);
          } else {
            ball_mp.control(ball_direction);
            ball_mp.setMotionProfileMode();
            ballPosition = Robot.m_ball.getBallRotateEncoder();
          }
        } else {
          Robot.m_ball._collectorRotateMotor.set(ControlMode.Position, target_position);
        }
         
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
    boolean bMPDone = climber_mp.isMotionProfileDone();
    boolean bMPOtherSideDone = true;
    if (climberPod.getSide() == PodPosition.RIGHT) {
      bMPOtherSideDone = Robot.m_climberPodFrontLeft.isMotionProfileFinished();
    } else {
      bMPOtherSideDone = Robot.m_climberPodFrontRight.isMotionProfileFinished();
    }
    bMPDone = (bMPDone && bMPOtherSideDone);
    boolean bMPBallDone = false;
    boolean bCommandDone = false;
    if (direction == ClimberDirection.UP && manual_override) { 
      bCommandDone = bMPDone;
    } else {
      if (climberPod.getSide() == PodPosition.LEFT && ball_direction == CollectorDirection.UP) {
        bMPBallDone = ball_mp.isMotionProfileDone();
        bCommandDone = (bMPDone && bMPBallDone);
      } else {
        bCommandDone = bMPDone;
      }
    }
    if (elapsedTime >= watchDogTime) {
      System.out.println("Watch Dog timer popped.");
      return true;
    }

    return bCommandDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber_mp.stopMotionProfile();
    if (climberPod.getSide() == PodPosition.LEFT && !manual_override) {
      if (ball_direction == CollectorDirection.UP) {
        ball_mp.stopMotionProfile();
      } else {
        Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
      }
    }
    // mp.stopWorking();
    System.out.println("MotionProfileTestClimberDouble(): End");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    climber_mp.stopMotionProfile();
    if (climberPod.getSide() == PodPosition.LEFT && !manual_override) {
      if (ball_direction == CollectorDirection.UP) {
        ball_mp.stopMotionProfile();
      } else {
        Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
      }
    }
    // mp.stopWorking(); //only used by threading alternative
    System.out.println("MotionProfileTestClimberDouble(): Interrupted");
  }

}