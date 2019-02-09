package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

// Command used to toggle the neck state
public class ToggleNeck extends Command {

  // Command Constructor
  public ToggleNeck() {
    // Declare subsystem dependencies
    // requires(Robot.m_hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.m_hatch.toggleNeck();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
