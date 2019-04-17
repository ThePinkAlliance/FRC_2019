/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CloseBeak extends Command {

  public Timer waitTime = null;
  public double endTime = 2;

  public CloseBeak() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_hatch);

     waitTime = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    waitTime.reset();
    waitTime.start();

    // Close beak
    Robot.m_hatch._beak.set(false);

    //push pushers
    Robot.m_hatch._push.set(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // If limit switch is pressed, finished
    if ((!Robot.m_hatch.leftLimitSwitchHatchCollected.get() || !Robot.m_hatch.rightLimitSwitchHatchCollected.get()) 
        || waitTime.get() <= endTime ){    
      return false;
    } else {
      Robot.m_hatch._push.set(false);
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
