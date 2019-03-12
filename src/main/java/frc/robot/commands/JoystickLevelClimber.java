/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.MotionProfileClimber;

public class JoystickLevelClimber extends Command {


  private Joystick js = null; 

  public JoystickLevelClimber() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climberPodFrontLeft);
    requires(Robot.m_climberPodBackLeft);
    requires(Robot.m_climberPodBackRight);
    requires(Robot.m_climberPodFrontRight);

    js = Robot.m_oi.getTowerJoystick();

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (js != null) {

      double yAdjust = js.getRawAxis(OI.rightStick);
      double xAdjust = js.getRawAxis(OI.RXAxis);

      
      if (xAdjust > 0) {
        Robot.m_climberPodBackRight.set(xAdjust*xAdjust*.5);
        Robot.m_climberPodFrontRight.set(xAdjust*xAdjust*.5);
      }
      else if (xAdjust < 0) {
        Robot.m_climberPodBackLeft.set(xAdjust*xAdjust*.5);
        Robot.m_climberPodFrontLeft.set(xAdjust*xAdjust*.5);
      }
      if (yAdjust > 0) {
        Robot.m_climberPodFrontLeft.set(yAdjust*yAdjust*.5);
        Robot.m_climberPodFrontRight.set(yAdjust*yAdjust*.5);
      }
      else if (yAdjust < 0) {
        Robot.m_climberPodBackLeft.set(yAdjust*yAdjust*.5);
        Robot.m_climberPodBackRight.set(yAdjust*yAdjust*.5);
      }
        
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
    Robot.m_climberPodFrontLeft.set(0);
    Robot.m_climberPodBackLeft.set(0);
    Robot.m_climberPodBackRight.set(0);
    Robot.m_climberPodFrontRight.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_climberPodFrontLeft.set(0);
    Robot.m_climberPodBackLeft.set(0);
    Robot.m_climberPodBackRight.set(0);
    Robot.m_climberPodFrontRight.set(0);
  }
}
