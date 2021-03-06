package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.utils.Gains;

// Subsystem used for defining DriveTrain hardware and methods
public class DriveTrain extends Subsystem {

  // Define Motor controllers
  private CANSparkMax _rightFront = new CANSparkMax(RobotMap.rightFrontMotorPort, MotorType.kBrushless);
  private CANSparkMax _rightRear = new CANSparkMax(RobotMap.rightBackMotorPort, MotorType.kBrushless);
  private CANSparkMax _leftFront = new CANSparkMax(RobotMap.leftFrontMotorPort, MotorType.kBrushless);
  private CANSparkMax _leftRear = new CANSparkMax(RobotMap.leftBackMotorPort, MotorType.kBrushless);

  // Define DifferentialDrive for this Subsystem
  private DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rightFront);

  // Define encoders
  CANEncoder _enc_leftRear = new CANEncoder(_leftRear);
  CANEncoder _enc_leftFront = new CANEncoder(_leftFront);
  CANEncoder _enc_rightRear = new CANEncoder(_rightRear);
  CANEncoder _enc_rightFront = new CANEncoder(_rightFront);

  // Declare class variables
  public double leftGoverned = 0.0;
  public double rightGoverned = 0.0;
  public double baseKp = 0.3;
  public double baseKd = 0.0;
  private AHRS _ahrs = new AHRS(SPI.Port.kMXP);
  public static final double NAVX_BAD_VALUE = -989898.0;

  // Subsystem Constructor
  public DriveTrain() {

    // Set up Subsystem Hardware
    _leftFront.setInverted(false);
    _rightRear.setInverted(false);
    _leftRear.setInverted(false);
    _rightRear.follow(_rightFront);
    _leftRear.follow(_leftFront);

    // Reset Gyro
    resetGyro();
  }

  // Method to define the default command for the DriveTrain
  @Override
  public void initDefaultCommand() {
  
    // Default command: JoystickDrive
    setDefaultCommand(new JoystickDrive());
  }


  // Method that will stop the Drive Train's Movement
  public void stopDriveTrain() {
    _diffDrive.tankDrive(0,0);
  }

  // Method that will return the current encoder position of the front right motor
  public double getFrontRightPosition() {
    if (_enc_rightFront != null)
       return _enc_rightFront.getPosition();
    else
       return 0.0;
  }

  // Method that will return the current encoder position of the front left motor
  public double getFrontLeftPosition() {
    if (_enc_leftFront != null)
       return _enc_leftFront.getPosition();
    else
       return 0.0;
  }

  // Method that returns the current angle of the robot as determined by the navx
  public double getGyroAngle() {
    if (_ahrs != null)
       return this._ahrs.getAngle();
    else
       return NAVX_BAD_VALUE;

  }

  // Method that returns the current yaw of the robot as determined by the navx
  public double getGyroYaw() {
    if (_ahrs != null)
       return this._ahrs.getYaw();
    else
       return NAVX_BAD_VALUE;
  }

  // Method that resets the gyro to 0
  public void resetGyro() {
    if (_ahrs != null)
       this._ahrs.reset();
  }

  // Method that sets the right motors to a value from -1 to 1
  public void rightMotor(double output) {
    _rightFront.set(output);
  }

  // Method that sets the left motors to a value from -1 to 1
  public void leftMotor(double output) {
    _leftFront.set(output);
  }

  // Method that stops the right motors
  public void rightMotorStop() {
    _rightFront.set(0);
  }

  // Method that stops the left motors
  public void leftMotorStop() {
    _leftFront.set(0);
  }

  // Method that stops the drive train motors
  public void drivetrainStop() {
    rightMotorStop();
    leftMotorStop();
  }

  // Based on angle error, returns pos or neg error
  public double get_left_angle_error(double desired_angle, double starting_angle) {
    double target_angle = desired_angle + starting_angle;
    double left_angle_error = (target_angle - _ahrs.getAngle());
    if (desired_angle > 0) {
      return left_angle_error;
    } else if (desired_angle < 0) {
      return -left_angle_error;
    } else {
      return 0;
    }
  }

  // Based on angle error, returns pos or neg error
  public double get_right_angle_error(double desired_angle, double starting_angle) {
    double target_angle = desired_angle + starting_angle;
    double right_angle_error = (target_angle - _ahrs.getAngle());
    if (desired_angle > 0) {
      return -right_angle_error;
    } else if (desired_angle < 0) {
      return right_angle_error;
    } else {
      return 0;
    }
  }

   // Method that drives the robot a set distance
   public boolean drive_to_distance(double target_distance, double left_starting_position, double right_starting_position) {
    double left_motor_command = (baseKp * (left_starting_position - target_distance) + 
                                (baseKd * _enc_leftFront.getVelocity()));
      double right_motor_command = (baseKp * (right_starting_position - target_distance) +
                                  (baseKd * _enc_rightFront.getVelocity()));
    _diffDrive.tankDrive(left_motor_command, right_motor_command);

    return (((target_distance + left_starting_position) - _enc_leftFront.getPosition()) < 0 && 
           ((target_distance + right_starting_position) - _enc_rightFront.getPosition()) < 0);
  }

  // Method that turns to a set angle
  public boolean turn_to_angle (double target_angle) {
    double starting_angle = _ahrs.getAngle();
    double left_motor_command = ((baseKp * get_left_angle_error(target_angle, starting_angle)) + (baseKd *_ahrs.getAngle()));
    double right_motor_command = ((baseKp * get_right_angle_error(target_angle, starting_angle) + (baseKd *_ahrs.getAngle())));

    _diffDrive.tankDrive(left_motor_command, right_motor_command);

    return (target_angle + starting_angle) - _ahrs.getAngle() == 0;
  }

  // Method to turn to an angle and then drive a set distance
  public boolean drive_to_position(double target_distance, double target_angle) {
    turn_to_angle(target_angle);
    if (turn_to_angle(target_angle)) {
      drive_to_distance(target_distance, getFrontLeftPosition(), getFrontRightPosition());
    } else {
      turn_to_angle(target_angle);
    }

    return drive_to_distance(target_distance, getFrontLeftPosition(), getFrontRightPosition()) && turn_to_angle(target_angle);
  }
  
  // Method to drive based on joysticks while accounting for governor
  public void tankDriveByJoystick(double left, double right) {

    // Invert motor direction and account for gain
    leftGoverned = left * left;
    rightGoverned = right * right;
    if (left < 0) {
      leftGoverned = leftGoverned * -1;
    }
    if (right < 0) {
      rightGoverned = rightGoverned * -1;
    }
    leftGoverned = -left * Gains.baseMotorGain;
    rightGoverned = -right * Gains.baseMotorGain;

    // Add Telemetry 
    //system..out.println("||----------Drive Train----------||");
    //system..out.println("Left: " + leftGoverned +  " ---    Right: " + rightGoverned);

    // Set Motor Power
    _diffDrive.tankDrive(leftGoverned, rightGoverned);
	}
}