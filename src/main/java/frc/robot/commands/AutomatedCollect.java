package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutomatedCollect extends Command {
  private Timer timer = null;
  public AutomatedCollect() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_ball);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer = new Timer();
    timer.start();
    Robot.m_ball.collect();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.m_ball.getCollectState()) {
      Robot.m_ball.collect();
      Robot.m_ball.setRotateMotorCmd(Robot.m_ball.COLLECT_POS);
    } else if (!Robot.m_ball.getCollectState()) {
      Robot.m_ball.hold();
      Robot.m_ball.setRotateMotorCmd(Robot.m_ball.CARGO_POS);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.m_ball.getBallRotateEncoder() - Robot.m_ball.CARGO_POS) <= 100 && (timer.get() > 0.5)) || (Math.abs(Robot.m_oi.getTowerJoystick().getRawAxis(1)) > 0.05);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_ball.hold();
    Robot.m_ball.setRotateMotorCmd(Robot.m_ball.CARGO_POS);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_ball.hold();
    Robot.m_ball.setRotateMotorCmd(Robot.m_ball.getBallRotateEncoder());
  }
}
