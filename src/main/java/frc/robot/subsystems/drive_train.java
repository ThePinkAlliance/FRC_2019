package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.JoystickDrive;
import frc.robot.RobotMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * Add your docs here.
 */
public class drive_train extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final int ESC_FRONT_RIGHT = 3;
  public static final int[] ENC_DIO_FRONT_RIGHT = {4,5};

  public static final int ESC_FRONT_LEFT = 4;
  public static final int[] ENC_DIO_FRONT_LEFT = {6,7};

  public static final int ESC_REAR_RIGHT = 2;
  public static final int[] ENC_DIO_REAR_RIGHT = {2,3};

  public static final int ESC_REAR_LEFT = 1;
  public static final int[] ENC_DIO_REAR_LEFT = {0,1};

  public static final boolean ENC_INVERT_COUNT_FALSE = false;
  public static final boolean ENC_INVERT_COUNT_TRUE = true;

  //Encodering distance per pulse: No gearing to account for:
  public static final double WHEEL_DIAMETER = 6.0;
  public static final double PULSE_PER_REVOLUTION = 250;  //Need to revisit this value!!
  public final double DISTANCE_PER_PULSE = (double)(Math.PI*WHEEL_DIAMETER)/PULSE_PER_REVOLUTION;


  private CANSparkMax _rightFront = null;
  private CANSparkMax _rightRear = null;
  private CANSparkMax _leftFront = null;
  private CANSparkMax _leftRear = null;
  //private Faults _faults_L = new Faults();
  //private Faults _faults_R = new Faults();
  private DifferentialDrive _diffDrive = null;
  private double _governor = 1.0;

  // private CANEncoder _enc_leftRear = null;
  private CANEncoder _enc_leftFront = null;
  // private CANEncoder _enc_rightRear = null;
  private CANEncoder _enc_rightFront = null;
  private double  _enc_Kp = 1.0;
  private double  _enc_Ki = 0.0;
  private double  _enc_Kd = 0.0;

  public drive_train() {

    //Motor setup
    _rightFront = new CANSparkMax(RobotMap.rightFrontMotorPort, MotorType.kBrushless);
    _rightRear = new CANSparkMax(RobotMap.rightBackMotorPort, MotorType.kBrushless);
    _leftFront = new CANSparkMax(RobotMap.leftFrontMotorPort, MotorType.kBrushless);
    _leftRear = new CANSparkMax(RobotMap.leftBackMotorPort, MotorType.kBrushless);
    _rightFront.setInverted(false);
    _leftFront.setInverted(true);
    _rightRear.setInverted(false);
    _leftRear.setInverted(true);
    
    //_faults_L = new Faults();
    //_faults_R = new Faults();
    _rightRear.follow(_rightFront);
    _leftRear.follow(_leftFront);

   
    //Encoder setup
    _enc_leftFront = new CANEncoder(_leftFront);
    // _enc_leftRear = new CANEncoder(_leftRear);
    _enc_rightFront = new CANEncoder(_rightFront);
    // _enc_rightRear = new CANEncoder(_rightRear);

    _diffDrive = new DifferentialDrive(_leftFront, _rightFront);
    _diffDrive.setRightSideInverted(false);
    

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new JoystickDrive());
  }

  public void setGovernor(double percent) {
    this._governor = percent;
  }

  public double getGovernor() {
    return this._governor;
  }

  public void stopDriveTrain() {
    _diffDrive.tankDrive(0,0);
  }

  public double getFrontRightPosition() {
    if (_enc_rightFront != null)
       return _enc_rightFront.getPosition();
    else
       return 0.0;
  }

  public double getFrontLeftPosition() {
    if (_enc_leftFront != null)
       return _enc_leftFront.getPosition();
    else
       return 0.0;
  }
  public double getFrontDistanceAverage() {
    double left = getFrontLeftPosition();
    double right = getFrontRightPosition();
    //Make this more resilient if needed (e.g. base it off of single encoder)
    //in case one of them is null
    return (double)((left + right) / 2.0);
  }

  public void setEncKp(double value) {
    this._enc_Kp = value;
  }

  public void setEncKi(double value) {
    this._enc_Ki = value;
  }

  public void setEncKd(double value) {
    this._enc_Kd = value;
  }

  public double getEncKp() {
    return this._enc_Kp;
  }

  public double getEncKi() {
    return this._enc_Ki;
  }

  public double getEncKd() {
    return this._enc_Kd;
  }

  public void rightMotor(double output) {
    _rightFront.set(output);
  }

  public void leftMotor(double output) {
    _leftFront.set(output);
  }

  public void rightMotorStop() {
    _rightFront.set(0);
  }

  public void leftMotorStop() {
    _leftFront.set(0);
  }

  public void tankDriveByEncoder(double left, double right) {
    
		_diffDrive.tankDrive(left, right);
	}
  public void tankDriveByJoystick(double left, double right) {
    //System.out.println("Left: " + leftSpeed + " <===>  Right: " + rightSpeed);
    //For this setup (ESC forward green), If LEFT negative make positive, if positive make negative
    //For this setup (ESC forward green), If RIGHT positive make negative, if negative make positive
    left = left < 0 ? left*-1 : -left;
    right = right > 0 ? right*-1 : -right;
    //Apply governor for safety.  This brute safety needs to be taken into account
    //by either removing it or using it when tuning PID controllers, etc.
    double leftGoverned = left * Math.abs(_governor);
    double rightGoverned = right * Math.abs(_governor);
    //System.out.println("Left: " + leftGoverned +  " ---    Right: " + rightGoverned);
		_diffDrive.tankDrive(leftGoverned, rightGoverned);
	}
}