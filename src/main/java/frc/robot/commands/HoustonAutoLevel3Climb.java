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
import frc.robot.subsystems.Ball;
import frc.robot.subsystems.HoustonClimber;
import frc.robot.subsystems.HoustonClimber.ClimbLevel;
import frc.robot.subsystems.HoustonClimber.PodAction;
import frc.robot.subsystems.utils.HoustonMotionProfileBallExecutor;
import frc.robot.subsystems.utils.HoustonMotionProfileExecutor;
import frc.robot.Robot;



public class HoustonAutoLevel3Climb extends Command {

  private HoustonMotionProfileExecutor mp = null;
  private HoustonMotionProfileBallExecutor ball_mp = null;
  private PodAction action = PodAction.CLIMB;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private HoustonClimber climberPod = null;
  private Ball ball = null;
  private ClimbLevel level = ClimbLevel.LEVEL3;
  private double preBallProfileEncoderPosition = 0.0;  //Ball encoder position at start of profile, before resetting to 0
  private boolean climberPositionRead = false;
  private double climberPosition = 0.0;
  private boolean ballPositionRead = false;
  private double ballPosition = 0.0;


  public HoustonAutoLevel3Climb(PodAction action, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    requires(Robot.m_ball);
    this.climberPod = Robot.m_climber;
    this.ball = Robot.m_ball;

    //set the direction
    this.action = action;

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
        //POD
        mp = climberPod.getMP();
        climberPod.setAction(action);
        mp.reset();
        climberPod.resetEncoderPosition(0);
        mp.setMotionProfileMode();
        mp.startMotionProfile();
        System.out.println("HoustonMotionProfileExecutor(): initialized");
        //BALL
        ball_mp = ball.getMP();
        ball_mp.reset();
        preBallProfileEncoderPosition = Robot.m_ball.getBallRotateEncoder();
        Robot.m_ball.resetEncoderPosition(0);
        ball_mp.setMotionProfileMode();
        ball_mp.startMotionProfile();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if (climberPod.isMotionProfileFinished()) {
        //simply hold your position
        if (climberPositionRead == false) {
          climberPosition = climberPod.getEncPositionMaster();
          climberPositionRead = true;
        } else {
          climberPod.setPosition(climberPosition);
        }
      } else {
         mp.control(action, level);
         mp.setMotionProfileMode();
      }

      if (ball.isMotionProfileFinished()) {
        //simply hold your position
        if (ballPositionRead == false) {
          ballPosition = ball.getBallRotateEncoder();
          ballPositionRead = true;
        } else {
          ball._collectorRotateMotor.set(ControlMode.Position, ballPosition);
        }
      } else {
         ball_mp.control();
         ball_mp.setMotionProfileMode();
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double elapsedTime = watchDog.get();
    boolean bMPDone = mp.isMotionProfileDone();
    boolean bMPBallDone = ball_mp.isMotionProfileDone();
    
    if (elapsedTime >= watchDogTime) {
      System.out.println("Watch Dog timer popped.");
      return true;
    }

    return (bMPDone && bMPBallDone);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mp.stopMotionProfile();
    climberPod.set(0.1);
    ball_mp.stopMotionProfile();
    System.out.println("Houston Climb: End");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mp.stopMotionProfile();
    ball_mp.stopMotionProfile();
    System.out.println("Houston Climb: Interrupted");
  }
}
