/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorPIDControl extends Command {
  
  private Timer watchDogTimer = null;
  CANPIDController _elevatorPIDController = null;
  double targetPos = 0.0;
  private double watchDogTime = 0.0;

  public ElevatorPIDControl(double targetPosition) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_elevator);
    _elevatorPIDController = Robot.m_elevator._elevator.getPIDController();
    targetPos = targetPosition;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _elevatorPIDController.setReference(targetPos, ControlType.kPosition);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    boolean bTimerPopped = false;

    double elapsedTime = watchDogTimer.get();


    if ( elapsedTime >= watchDogTime ) {
      bTimerPopped = true;
    }
    return bTimerPopped;
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
