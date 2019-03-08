/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ToggleBeakByLimitSwitch extends Command {
  public ToggleBeakByLimitSwitch() {

    // Requires hatch subsystem
    requires(Robot.m_hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Evan said he wants neck to be down when beak is opened
    // TODO: test this, is false up?
    //if (!Robot.m_hatch.isNeckUp())
      //Robot.m_hatch.toggleNeck();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // If limit switch is pressed, open the beak
    if (!Robot.m_hatch.leftLimitSwitchHatchCollected.get() || !Robot.m_hatch.rightLimitSwitchHatchCollected.get()) {
      Robot.m_hatch._beak.set(false); // Open Beak
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
