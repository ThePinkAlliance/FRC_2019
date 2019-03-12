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
import frc.robot.subsystems.MotionProfileClimberMaster;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;


public class JoystickLevelClimber extends Command {


  private Joystick js = null; 
  private MotionProfileClimberMaster climber = null;

  public JoystickLevelClimber() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climberMaster);
    
    this.climber = Robot.m_climberMaster;

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
      boolean climbTrue = js.getRawButton(OI.leftTriggerButtonNumber);
      

      double frx = 0;
      double flx = 0;
      double brx = 0;
      double blx = 0;

      double fry = 0;
      double fly = 0;
      double bry = 0;
      double bly = 0;
      
      if (xAdjust > 0) {
        blx += xAdjust*xAdjust;
        flx += xAdjust*xAdjust;
      }
      else if (xAdjust < 0) {
        brx += xAdjust*xAdjust;
        frx += xAdjust*xAdjust;
      }
      if (yAdjust > 0) {
        fly += yAdjust*yAdjust;
        fry += yAdjust*yAdjust;
      }
      else if (yAdjust < 0) {
        bly += yAdjust*yAdjust;
        bry += yAdjust*yAdjust;
      }

      if (climbTrue) {
        climber.set(PodPosition.FRONT, PodPosition.RIGHT, ((frx+fry)/2)+.2);
        climber.set(PodPosition.FRONT, PodPosition.LEFT, ((flx+fly))+.5);
        climber.set(PodPosition.BACK, PodPosition.RIGHT, ((brx+bry))/2+.2);
        climber.set(PodPosition.BACK, PodPosition.LEFT, (blx+bly)+.5);
      }
      else {
        climber.set(PodPosition.FRONT, PodPosition.RIGHT, (frx+fry)/2);
        climber.set(PodPosition.FRONT, PodPosition.LEFT, (flx+fly)/2);
        climber.set(PodPosition.BACK, PodPosition.RIGHT, (brx+bry)/2);
        climber.set(PodPosition.BACK, PodPosition.LEFT, (blx+bly));
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
    climber.set(PodPosition.FRONT, PodPosition.RIGHT, 0);
    climber.set(PodPosition.FRONT, PodPosition.LEFT, 0);
    climber.set(PodPosition.BACK, PodPosition.RIGHT, 0);
    climber.set(PodPosition.BACK, PodPosition.LEFT, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    climber.set(PodPosition.FRONT, PodPosition.RIGHT, 0);
    climber.set(PodPosition.FRONT, PodPosition.LEFT, 0);
    climber.set(PodPosition.BACK, PodPosition.RIGHT, 0);
    climber.set(PodPosition.BACK, PodPosition.LEFT, 0);
  }
}
