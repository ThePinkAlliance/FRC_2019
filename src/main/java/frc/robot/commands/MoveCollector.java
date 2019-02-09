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

public class MoveCollector extends Command {
  private Joystick js = null;
  public CollectorTargetPosition position;
  private double stickValue = 0;
  private double CollectPosition = 0;
  private double RocketPosition = 50;
  private double CargoPosition = 150;
  private double StowPosition = 200;

  public enum CollectorTargetPosition {
    COLLECT,
    ROCKET,
    CARGO,
    STOW,
    JOYSTICK;
  }

  public MoveCollector(CollectorTargetPosition position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.m_ball);
    js = Robot.m_oi.getBaseJoystick();
    stickValue = js.getRawAxis(OI.LXAxis);
    this.position = position;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // RotateToPosition();
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

  // public void RotateToPosition() {
  //   switch (position) {
  //     case COLLECT:
  //       Robot.m_ball.rotateToPosition(CollectPosition, Robot.m_ball.getCurrentPosition());
  //       if (Robot.m_ball.getCurrentPosition() == CollectPosition || 0.1 < stickValue || -0.1 > stickValue) {
  //         position = CollectorTargetPosition.JOYSTICK;
  //       } else {
  //         position = CollectorTargetPosition.COLLECT;
  //       }
  //       break;

  //     case ROCKET:
  //       Robot.m_ball.rotateToPosition(RocketPosition, Robot.m_ball.getCurrentPosition());
  //       if (Robot.m_ball.getCurrentPosition() == RocketPosition || 0.1 < stickValue || -0.1 > stickValue) {
  //         position = CollectorTargetPosition.JOYSTICK;
  //       } else {
  //         position = CollectorTargetPosition.ROCKET;
  //       }
  //       break;

  //     case CARGO:
  //       Robot.m_ball.rotateToPosition(CargoPosition, Robot.m_ball.getCurrentPosition());
  //       if (Robot.m_ball.getCurrentPosition() == CargoPosition || 0.1 < stickValue || -0.1 > stickValue) {
  //         position = CollectorTargetPosition.JOYSTICK;
  //       } else {
  //         position = CollectorTargetPosition.CARGO;
  //       }
  //       break;

  //     case STOW:
  //       Robot.m_ball.rotateToPosition(StowPosition, Robot.m_ball.getCurrentPosition());
  //       if (Robot.m_ball.getCurrentPosition() == StowPosition || 0.1 < stickValue || -0.1 > stickValue) {
  //         position = CollectorTargetPosition.JOYSTICK;
  //       } else {
  //         position = CollectorTargetPosition.STOW;
  //       }
  //       break;

  //     case JOYSTICK:
  //       ReadJoystick();
  //       break;
  //   }
  // }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

   // Method to set motor power based of the stickValue
   public void ReadJoystick() {
    // Read out stickValue
    if (js != null) {
      if(stickValue > -0.1 && stickValue < 0.1) {
        stickValue = 0;
      }
      // Set _elevator Motor to stickValue
      // Robot.m_ball.moveBall(stickValue);
    }
  }
}
