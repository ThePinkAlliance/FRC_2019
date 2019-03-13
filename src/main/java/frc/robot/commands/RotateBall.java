/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;

public class RotateBall extends Command {

  // Init sticks
  private Joystick js = null;
  private double stickValue = 0;
  RotateToPosition target_position;

  public enum RotateToPosition {
    COLLECT,
    LOW_ROCKET,
    CARGO,
    JOYSTICK;
  }

  public RotateBall(RotateToPosition target_position) {

    // Requires Ball collector
    requires(Robot.m_ball);

    // Define sticks
    js = Robot.m_oi.getTowerJoystick();
    stickValue = js.getRawAxis(OI.LXAxis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch(target_position) {
      case COLLECT:
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.COLLECT_POS);
        if (stickValue > 0.1 || stickValue < -0.1) {
          target_position = RotateToPosition.JOYSTICK;
        }

      case LOW_ROCKET:
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.LOW_ROCKET_POS);
        if (stickValue > 0.1 || stickValue < -0.1) {
          target_position = RotateToPosition.JOYSTICK;
        }
        
      case CARGO:
        Robot.m_ball.setRotateMotorCmd(Robot.m_ball.CARGO_POS);
        if (stickValue > 0.1 || stickValue < -0.1) {
          target_position = RotateToPosition.JOYSTICK;
        }

      case JOYSTICK:
        ReadJoystick();
    }
    //system..out.println("Tower Rotate Pos: " + Robot.m_ball._enc_collectorRotate);
    Robot.m_ball._collectorMotor.set(Robot.m_ball.holdSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_ball._collectorRotateMotor.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_ball._collectorRotateMotor.set(0);
  }

   // Method to set motor power based of the stickValue
   public void ReadJoystick() {
    // Read out stickValue
    if (js != null) {
      if(stickValue > -0.1 && stickValue < 0.1) {
        stickValue = 0;
      }
      // Set _elevator Motor to stickValue
      Robot.m_ball.moveBall(stickValue);
    }
  }
}
