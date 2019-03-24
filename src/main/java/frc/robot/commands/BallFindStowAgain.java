package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class BallFindStowAgain extends Command {
  private Timer timer = null;
  private double max_time = 0.0;

  private double move_power = 0.0;
 
  public BallFindStowAgain(double movePower, double watchdogTimer) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_ball);
    
    
    max_time = watchdogTimer;
    move_power = movePower;
    
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
    
      Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, move_power);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.get() >= max_time;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
      Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
      Robot.m_ball._collectorRotateMotor.setSelectedSensorPosition(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
      Robot.m_ball._collectorRotateMotor.setSelectedSensorPosition(0); 
  }
}
