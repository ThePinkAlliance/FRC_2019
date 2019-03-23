package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class MotionProfileClimberHold extends Command {

  private Timer watchDog = null;
  private double watchDogTime = 0.0;

  private double rightPodPositionToHold = 0.0;
  private double leftPodPositionToHold = 0.0;
  private int collectorPositionToHold = 0;
  private double p_gain = 0.0;
  

  public MotionProfileClimberHold(double watchDogTime, double ballGain) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climberPodFrontLeft);
    requires(Robot.m_climberPodFrontRight);
    requires(Robot.m_ball);

    p_gain = ballGain;
    
    this.watchDogTime = watchDogTime;

    watchDog = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    //start the timer
    watchDog.reset();
    watchDog.start();

    rightPodPositionToHold = Robot.m_climberPodFrontRight.getEncPosition();
    leftPodPositionToHold = Robot.m_climberPodFrontLeft.getEncPosition();
    collectorPositionToHold = Robot.m_ball._collectorRotateMotor.getSelectedSensorPosition();


    Robot.m_climberPodFrontRight.setPosition(rightPodPositionToHold);
    Robot.m_climberPodFrontLeft.setPosition(leftPodPositionToHold);
    Robot.m_ball.setClimberRotateMotorCmd(collectorPositionToHold, p_gain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_climberPodFrontRight.setPosition(rightPodPositionToHold);
    Robot.m_climberPodFrontLeft.setPosition(leftPodPositionToHold);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    double elapsedTime = watchDog.get();

    if (elapsedTime >= watchDogTime) {
      return true;
    }
    
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
 
    //set climber pod motor to zero
    Robot.m_climberPodFrontRight.set(0);
    Robot.m_climberPodFrontLeft.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

    //set climber pod motor to zero
    Robot.m_climberPodFrontRight.set(0);
    Robot.m_climberPodFrontLeft.set(0);
  }
}
