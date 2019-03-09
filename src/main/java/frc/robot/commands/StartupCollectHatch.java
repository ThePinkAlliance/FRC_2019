/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StartupCollectHatch extends Command {
  public StartupCollectHatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_hatch);
    requires(Robot.m_elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_elevator.moveElevatorToPosition(-21);
    if (!Robot.m_hatch.leftLimitSwitchHatchCollected.get() || !Robot.m_hatch.rightLimitSwitchHatchCollected.get()) {
      Robot.m_hatch._beak.set(false); // Open Beak
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (!Robot.m_hatch.leftLimitSwitchHatchCollected.get() || !Robot.m_hatch.rightLimitSwitchHatchCollected.get() || Robot.m_elevator._enc_elevator.getPosition() == -20) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_hatch._neck.set(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
