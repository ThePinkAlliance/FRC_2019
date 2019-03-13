/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MotionProfileClimberMaster;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;

public class MotionProfileClimberMasterTest extends Command {

  private Joystick js = null; 
  private MotionProfileClimberDouble mp = null;
  private ClimberDirection direction = ClimberDirection.UP;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private final double UNWIND_TIME = 0.0;  //one sec to let talon unwind
  private double doneTime = 0;
  private MotionProfileClimberMaster climberPod = null;
  private PodPosition location;

  private Timer profileStartTimer = null;
  private double delayTime = 0;
  private double moveVoltage = 0;
  private boolean motionProfileStarted = false;
  public double Kf_FL = 1.0;//2.0;
  public double Kf_BR = 1.0;//1.0;
  public double Kf_BL = 1.9;//2.0;
  //==========================
  public double Kp_FL = 0.2 / 1000.0;
  public double Kp_BR = 0.8 / 1000.0;
  public double Kp_BL = 0.5 / 1000.0;
  public double masterPower = 0.0;//climberPod.getOutput(PodPosition.FRONT, PodPosition.RIGHT);
  public double masterPosition = 0.0;//climberPod.getEncPosition(PodPosition.FRONT, PodPosition.RIGHT);
  public double errorFL = 0.0;//masterPosition - climberPod.getEncPosition(PodPosition.FRONT, PodPosition.LEFT);
  public double errorBR = 0.0;//masterPosition - climberPod.getEncPosition(PodPosition.BACK, PodPosition.RIGHT);
  public double errorBL = 0.0;//masterPosition - climberPod.getEncPosition(PodPosition.BACK, PodPosition.LEFT);
  public double powerFL = 0.0;//(Kp_FL * errorFL) + (Kf_FL * masterPower);
  public double powerBR = 0.0;//(Kp_BR * errorFL) + (Kf_BR * masterPower);
  public double powerBL = 0.0;// (Kp_BL * errorFL) + (Kf_BL * masterPower);
  
  /**
   * 
   * @param direction which direction are we going UP or DOWN.  This affects
   *                  which motion profile is loaded
   * @param watchDogTime amount of time this command must complete in
   * 
   */
  public MotionProfileClimberMasterTest(ClimberDirection direction, double preLoadMove, double watchDogTime, double profileDelayTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climberMaster);
    climberPod = Robot.m_climberMaster;
    //set the direction
    this.direction = direction;

    //voltage to send to the motors before running profile
    this.moveVoltage = preLoadMove;
   
    //cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;

    //how long to move motors before starting profile
    this.delayTime = profileDelayTime;

    //new up the timer for later use
    watchDog = new Timer();
    profileStartTimer = new Timer();

    System.out.println("CLIMBER Constructor completed...");

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    //prep your time
    watchDog.reset();
    watchDog.start();

    if (direction == ClimberDirection.UP) {
      // go to 0 if we are going up
      
    } else {
      // go to max if we are going down
    }

    //get the motion profile object associated with the subsystem
    mp = climberPod.getMP();
    climberPod.setDirection(direction);
    mp.reset();
    climberPod.resetEncoderPosition(PodPosition.FRONT, PodPosition.RIGHT);
    climberPod.resetEncoderPosition(PodPosition.FRONT, PodPosition.LEFT);
    climberPod.resetEncoderPosition(PodPosition.BACK, PodPosition.RIGHT);
    climberPod.resetEncoderPosition(PodPosition.BACK, PodPosition.LEFT);

    //mp.setMotionProfileMode();
    //mp.startWorking(movingUp); //only used by threading alternative
   // mp.startMotionProfile();
    System.out.println("MotionProfileClimberMasterTest(): initialized");

    //start the timer to delay the command
    profileStartTimer.reset();
    profileStartTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // //get time elapsed
    //double delayElapsedTime = profileStartTimer.get();

    // //if timer popped and profile not started
    // if (delayElapsedTime >= delayTime && !motionProfileStarted)  {
    //   //set flag and start motion profile
    //   climberPod.set(0);
    //   climberPod.resetEncoderPosition(0);

    //   motionProfileStarted = true;
    //   mp.startMotionProfile();
    // }
    // else if (delayElapsedTime >= delayTime && motionProfileStarted) {
    //   //continue executing motion profile
       //mp.control(direction, location);
       //mp.setMotionProfileMode();
       masterPower = 0.35;//climberPod.getOutput(PodPosition.FRONT, PodPosition.RIGHT);
       //masterPosition = climberPod.getEncPosition(PodPosition.FRONT, PodPosition.RIGHT);
       masterPosition = climberPod.getEncVelocity(PodPosition.FRONT, PodPosition.RIGHT);
       errorFL = masterPosition - climberPod.getEncVelocity(PodPosition.FRONT, PodPosition.LEFT);
       errorBR = masterPosition - climberPod.getEncVelocity(PodPosition.BACK, PodPosition.RIGHT);
       errorBL = masterPosition - climberPod.getEncVelocity(PodPosition.BACK, PodPosition.LEFT);
      //  errorFL = masterPosition - climberPod.getEncPosition(PodPosition.FRONT, PodPosition.LEFT);
      //  errorBR = masterPosition - climberPod.getEncPosition(PodPosition.BACK, PodPosition.RIGHT);
      //  errorBL = masterPosition - climberPod.getEncPosition(PodPosition.BACK, PodPosition.LEFT);
       powerFL = (Kp_FL * errorFL) + (Kf_FL * masterPower);
       powerBR = (Kp_BR * errorBR) + (Kf_BR * masterPower);
       powerBL = (Kp_BL * errorBL) + (Kf_BL * masterPower);
       climberPod.set(PodPosition.FRONT, PodPosition.RIGHT, masterPower);
       climberPod.set(PodPosition.FRONT, PodPosition.LEFT, powerFL);
       climberPod.set(PodPosition.BACK, PodPosition.RIGHT, powerBR);
       climberPod.set(PodPosition.BACK, PodPosition.LEFT, powerBL);

    // }
    // else {
    //   //otherwise set voltage to pod
    //   this.climberPod.set(moveVoltage);
    // }
   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //boolean mpPressed = Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    //System.out.println("mpPressed: " + mpPressed);
    double elapsedTime = watchDog.get();
    boolean bMPDone = mp.isMotionProfileDone();
    if (elapsedTime >= watchDogTime) {
      System.out.println("Watch Dog timer popped.");
      return true;
    }

    return bMPDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mp.stopMotionProfile();
    //mp.stopWorking();
    System.out.println("MotionProfileTestClimberDouble(): End");
    climberPod.set(PodPosition.FRONT, PodPosition.LEFT, 0);
    climberPod.set(PodPosition.BACK, PodPosition.RIGHT, 0);
    climberPod.set(PodPosition.BACK, PodPosition.LEFT, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mp.stopMotionProfile();
   // mp.stopWorking();  //only used by threading alternative
   System.out.println("MotionProfileTestClimberDouble(): Interrupted");
   climberPod.set(PodPosition.FRONT, PodPosition.LEFT, 0);
    climberPod.set(PodPosition.BACK, PodPosition.RIGHT, 0);
    climberPod.set(PodPosition.BACK, PodPosition.LEFT, 0);
  }

}