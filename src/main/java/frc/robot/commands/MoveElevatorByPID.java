/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
// import frc.robot.subsystems.Utils.ElevatorPIDSource;


//class currently configured to move to specified target before watchdog.
public class MoveElevatorByPID extends Command implements PIDOutput{

  private Timer watchDogTimer = null;
  private PIDController heightController = null;
  // private ElevatorPIDSource elevatorPIDSource = null;

  //need to confirm these by dragging the elevator up and watching
  //encoder values
  private double MinimumEncoderValue = 0.0;
  private double MaximumEncoderValue = 100.0;

  private double MinimumOutputSpeed = -1.0;
  private double MaximumOutputSpeed = 1.0;

  private double CommandTolerance = 1;

  private double desiredHeight = 0.0;
  private double watchDogTime = 0.0;
  // private double desiredSpeed = 0.0;

  public MoveElevatorByPID(double targetHeight, double completeByTime, double speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_elevator);

    this.desiredHeight = targetHeight;
    this.watchDogTime = completeByTime;
    // this.desiredSpeed = speed;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    watchDogTimer = new Timer();
    watchDogTimer.reset();
    watchDogTimer.start();



    // double P = Robot.m_elevator.getElevKp();
    // double I = Robot.m_elevator.getElevKi();
    // double D = Robot.m_elevator.getElevKd();

    // elevatorPIDSource = new ElevatorPIDSource();

    // heightController = new PIDController(P, I, D, elevatorPIDSource, this);

    heightController.setInputRange(MinimumEncoderValue, MaximumEncoderValue);
    heightController.setOutputRange(MinimumOutputSpeed, MaximumOutputSpeed);
    heightController.setContinuous(false);
    heightController.setAbsoluteTolerance(CommandTolerance);
    heightController.disable();
    heightController.setSetpoint(desiredHeight);
    heightController.enable();
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
    heightController.disable();
    heightController.close();
    // Robot.m_elevator._elevator.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    heightController.disable();
    heightController.close();
    // Robot.m_elevator._elevator.set(0);
  }

  @Override
  public void pidWrite(double output) {

    // Robot.m_elevator._elevator.set(output);
  }
}
