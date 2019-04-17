/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.utils.LimeLightPID;

public class AimAtTarget extends Command implements PIDOutput{

  private Timer watchDogTimer = null;
  private double watchDogTime = 100;

  private LimeLightPID pidSource = null;

  private PIDController turnController = null;
  private Joystick js = Robot.m_oi.getBaseJoystick();



  public AimAtTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    System.out.println( "command called: constructor" );

    requires(Robot.m_driveTrain);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    System.out.println( "command called: init" );


    watchDogTimer = new Timer();
    watchDogTimer.reset();
    watchDogTimer.start();

    pidSource = new LimeLightPID();

    turnController = new PIDController(.004, 0, 0, pidSource, this);

    turnController.setInputRange(-29.8, 29.8);
    turnController.setAbsoluteTolerance(1);
    turnController.setContinuous(true);
    turnController.setSetpoint(0);
    turnController.enable();

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    double elapsedTime = watchDogTimer.get();

    if( !js.getRawButton(OI.leftBumperButtonNumber)) {
      return true;
    }
    if (elapsedTime > watchDogTime) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    turnController.disable();
    turnController.close();
    Robot.m_driveTrain.stopDriveTrain();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    turnController.disable();
    turnController.close();
    Robot.m_driveTrain.stopDriveTrain();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

  }

  @Override
  public void pidWrite(double output) {

    double left = 0.5;
    double right = 0.5;

    double targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double validTargets = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if ( targetArea > 0 && targetArea < 2 ) {
      left = .8;
      right = .8;
    }

    if ( validTargets == 0 ) {
      left = .6;
      right = .6;
    }


    left -= output;
    right += output;

    //ta 55

    System.out.println( ": OUTPUT: " + output + " LEFT: " + left + " RIGHT: " + right);
    Robot.m_driveTrain.rawTankDrive(left, right);


  }
}
