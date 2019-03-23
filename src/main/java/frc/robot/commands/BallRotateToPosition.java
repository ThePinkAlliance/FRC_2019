package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class BallRotateToPosition extends Command {
  private Timer timer = null;
  private double target_position = 0.0;
  private double max_time = 0.0;
  private double p_gain = 0.0;
  private boolean manualOverride = false;

  public BallRotateToPosition(double position, double proportionalGain, double watchdogTimer, boolean manual_override) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_ball);
    p_gain = proportionalGain;
    target_position = position;
    max_time = watchdogTimer;
    manualOverride = manual_override;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer = new Timer();
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!manualOverride) {
      Robot.m_ball.setClimberRotateMotorCmd(target_position, p_gain);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.get() >= max_time || manualOverride;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (!manualOverride) {
      Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    if (!manualOverride) {
      Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput,0);
    }  
  }
}
