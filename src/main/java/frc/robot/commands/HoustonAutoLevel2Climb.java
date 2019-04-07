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
import frc.robot.subsystems.utils.HoustonMotionProfileExecutor;
import frc.robot.subsystems.utils.PresetPositions;
import frc.robot.Robot;

public class HoustonAutoLevel2Climb extends Command {

  private HoustonMotionProfileExecutor mp = null;
  private PodAction action = PodAction.CLIMB;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private HoustonClimber climberPod = null;
  private Ball ball = null;
  private ClimbLevel level = ClimbLevel.LEVEL2;
  private double p_gain = 0.0015;
  private double ballPosition = PresetPositions.BALL_CLIMB_POSITION;

  public HoustonAutoLevel2Climb(PodAction action, double p_gain, double ballPosition, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    requires(Robot.m_ball);
    this.climberPod = Robot.m_climber;
    this.ball = Robot.m_ball;

    // set the direction
    this.action = action;

    //set the ball details
    this.p_gain = p_gain;
    this.ballPosition = ballPosition;

    // cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;

    // new up the timer for later use
    watchDog = new Timer();

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // prep your time
    watchDog.reset();
    watchDog.start();

    // get the motion profile object associated with the subsystem
    // POD
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
    Robot.m_ball.setClimberRotateMotorCmd(ballPosition, p_gain);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double elapsedTime = watchDog.get();
    boolean bMPDone = mp.isMotionProfileDone();

    bMPDone = (mp.isMotionProfileDone());
        //&& (Math.abs(Robot.m_ball.getBallRotateEncoder() - Robot.m_ball.CARGO_POS) <= 100));

    if (elapsedTime >= watchDogTime) {
      System.out.println("Watch Dog timer popped.");
      return true;
    }

    return (bMPDone);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mp.stopMotionProfile();
    climberPod.set(0.1);
    Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
    System.out.println("Houston Climb: End");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mp.stopMotionProfile();
    Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
    System.out.println("Houston Climb: Interrupted");
  }
}
