/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*---------------------*/
/* NOT BEING USED!!!   */
/*---------------------*/

/*
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ToggleBeak extends Command {
  public ToggleBeak() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_hatch._manualToggle = true;
    Robot.m_hatch.toggleBeak();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Robot.m_hatch._manualToggle = false;
    if(Robot.m_hatch._beak.get()) {
      if (!Robot.m_hatch.limitSwitchHatchCollected.get()) {
        return false;
      } else {
        return false;
      }  
    } else if (!Robot.m_hatch._beak.get()) {
      if (Robot.m_hatch.limitSwitchHatchCollected.get()) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
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

*/