package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OpenBeak extends Command {
  public OpenBeak() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Open the beak
    Robot.m_hatch._beak.set(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // If the limit switch is not pressed, not finished
    if (!Robot.m_hatch.leftLimitSwitchHatchCollected.get() || !Robot.m_hatch.rightLimitSwitchHatchCollected.get()) {
      return false;
    } else {
      return true;
    }  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
