/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;



import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MotionProfileClimber;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;


public class MotionProfileClimberHoldByPower extends Command {

  private Timer watchDog = null;
  private Timer phase1Timer = null;
  private double watchDogTime = 0.0;
  private double phase1Time = 0.0;

  private MotionProfileClimber climberPod = null;
  private double power1 = 0.0;
  private double power2 = 0.0;

  private boolean manual_override = false;
  private double ballPosition = 0;

  
  /**
   * 
   * @param direction which direction are we going UP or DOWN.  This affects
   *                  which motion profile is loaded
   * @param watchDogTime amount of time this command must complete in
   * 
   */
  public MotionProfileClimberHoldByPower(MotionProfileClimber theClimberPod, double power1, double power2, double phase1Time, double watchDogTime, boolean manual_override) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);
    if (theClimberPod.getSide() == PodPosition.LEFT && !manual_override) {
       requires(Robot.m_ball);
    }

    this.climberPod = theClimberPod;

    //voltage to send to the motors before running profile
    this.power1 = power1;
    this.power2 = power2;
   
    //cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;
    this.phase1Time = phase1Time;

    this.manual_override = manual_override;

    //new up the timer for later use
    watchDog = new Timer();
    phase1Timer = new Timer();
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    //prep your time
    watchDog.reset();
    watchDog.start();
    phase1Timer.reset();
    phase1Timer.start();

    ballPosition = Robot.m_ball._collectorRotateMotor.getSelectedSensorPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double phase1Elapsed = phase1Timer.get();
    if (phase1Elapsed <= phase1Time)
       climberPod.set(power1);
    else
       climberPod.set(power2);

    if (climberPod.getSide() == PodPosition.LEFT  && !manual_override) {
       Robot.m_ball._collectorRotateMotor.set(ControlMode.Position, ballPosition);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //boolean mpPressed = Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    //System.out.println("mpPressed: " + mpPressed);
    double elapsedTime = watchDog.get();
    if (elapsedTime >= watchDogTime) {
      System.out.println("Watch Dog timer popped.");
      return true;
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climberPod.set(0);
    if (climberPod.getSide() == PodPosition.LEFT  && !manual_override) {
      Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
    }

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
   climberPod.set(0);
   if (climberPod.getSide() == PodPosition.LEFT  && !manual_override) {
     Robot.m_ball._collectorRotateMotor.set(ControlMode.PercentOutput, 0);
    }
  }

}