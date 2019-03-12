package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveToDistance extends Command {

  private double distance = 0;
  private double watchDogTime = 10;
  // private double maxSpeed = 0.7;
  private double startLeft = 0.0;
  private double startRight = 0.0;
  private boolean bDistanceReached = false;
  private Timer watchDogTimer = null;

  public DriveToDistance(double distance, double watchDogTime, double maxSpeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);
    this.distance = distance;
    this.watchDogTime = watchDogTime;
    // this.maxSpeed = maxSpeed;
    watchDogTimer = new Timer();
    watchDogTimer.reset();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    watchDogTimer.start();
    startLeft = Math.abs(Robot.m_driveTrain.getFrontLeftPosition());
    startRight = Math.abs(Robot.m_driveTrain.getFrontRightPosition()); 
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

     bDistanceReached = Robot.m_driveTrain.drive_to_distance(distance, startLeft, startRight);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean bTimerPopped = false;
    double currentTime = watchDogTimer.get();

    if (currentTime >= watchDogTime)
       bTimerPopped = true;

    return (bTimerPopped || bDistanceReached);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.stopDriveTrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.stopDriveTrain();
  }
}
