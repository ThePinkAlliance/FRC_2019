package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Hold;
import frc.robot.subsystems.utils.ConstantsBall;
import frc.robot.subsystems.utils.MotionProfileBallDouble;
import frc.robot.subsystems.utils.PresetPositions;
import frc.robot.subsystems.utils.MotionProfileBallDouble.CollectorDirection;


public class MotionProfileBall extends Subsystem {

  // Init motor controllers
  public Spark _collectorMotor = null;

  public TalonSRX _collectorRotateMotor = null;
  //public WPI_TalonSRX _collectorRotateMotor = null;

  /** some example logic on how one can manage an MP */
  MotionProfileBallDouble _example = null;


  public double _enc_collectorRotate = 0.0;

  // Init digital inputs
  public DigitalInput collectedOpticalSwitch = null;

  // Setup values
  // public static double rotateKd = 0.005;
  public static double rotateKp = 0.01;
  public double collectSpeed = -1;
  public double holdSpeed = -0.2;
  public double ejectSpeed = 1;
  public double COLLECT_POS = PresetPositions.BALL_COLLECT_POSITION;
  public double LOW_ROCKET_POS = PresetPositions.BALL_LOW_ROCKET_POSITION;
  public double CARGO_POS = PresetPositions.BALL_CARGO_POSITION;

  public MotionProfileBall() {

    // Construct motor controllers
    _collectorMotor = new Spark(RobotMap.collectorMotorPort);
    
    //DIFFERENT - aidan look at this
    //_collectorRotateMotor = new WPI_TalonSRX(RobotMap.collectorRotateMotorPort);
    _collectorRotateMotor = new TalonSRX(RobotMap.collectorRotateMotorPort);
    
    _collectorRotateMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    _collectorRotateMotor.setNeutralMode(NeutralMode.Brake);

    // Construct digital inputs
    collectedOpticalSwitch = new DigitalInput(RobotMap.collectedOpticalSwitchPort);

    _example = new MotionProfileBallDouble(_collectorRotateMotor);
  }

  @Override
  public void initDefaultCommand() {
    // Set default command to be Hold
    setDefaultCommand(new Hold());
  }

  // Method to move the Collector based off the joystickValue
  public void moveBall(double joystickValue) {
    //_collectorRotateMotor.set(joystickValue);
    _collectorRotateMotor.set(ControlMode.PercentOutput, joystickValue);
    //system..out.println("Collector Rotate Value: "+ joystickValue);
  }

  public double getBallRotateEncoder() {
    return _collectorRotateMotor.getSelectedSensorPosition();
  }

  public void setRotateMotorCmd(double target_position) {
    double rotate_motor_command = 0.00005 * (target_position - getBallRotateEncoder());
    //_collectorRotateMotor.set(rotate_motor_command);
    _collectorRotateMotor.set(ControlMode.PercentOutput, rotate_motor_command);
  }

  // Method to rotate the ball collector to collect a ball
  public void collect() {
    _collectorMotor.set(collectSpeed);
    //system..out.println("Collector Power: " + _collectorMotor.get());
  }

  // Method to set ball collector to hold ball in
  public void hold() {
    _collectorMotor.set(holdSpeed);
  }

  // Method to rotate ball collector to eject ball
  public void eject() {
    _collectorMotor.set(ejectSpeed);
  }

  // Method to check to see if ball is held
  public boolean getCollectState() {
    return collectedOpticalSwitch.get();
  }

  // public void holdRotation() {

  // }
  public void setupTalon() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    _collectorRotateMotor.configFactoryDefault();
    _collectorRotateMotor.clearMotionProfileTrajectories(); // online
    _collectorRotateMotor.changeMotionControlFramePeriod(5);
    _collectorRotateMotor.setNeutralMode(NeutralMode.Brake);

    //FIRST PLACE TO SET INVERTED (DEFAULT)
    _collectorRotateMotor.setInverted(true);
    //_collectorRotateMotor.setSensorPhase(true);   //UNCOMMENT THIS OUT ONCE YOU KNOW THE ANSWER
    

    /* Configure Selected Sensor for Motion Profile */
    _collectorRotateMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, ConstantsBall.kPIDLoopIdx,
        ConstantsBall.kTimeoutMs);
    /*
     * Keep sensor and motor in phase, postive sensor values when MC LEDs are green
     */

    /**
     * Configure MotorController Neutral Deadband, disable Motor Controller when
     * requested Motor Output is too low to process
     */
    _collectorRotateMotor.configNeutralDeadband(ConstantsBall.kNeutralDeadband, ConstantsBall.kTimeoutMs);

    /* Configure PID Gains, to be used with Motion Profile */
    _collectorRotateMotor.config_kF(ConstantsBall.kPIDLoopIdx, ConstantsBall.kGainsBall.kF, ConstantsBall.kTimeoutMs);
    _collectorRotateMotor.config_kP(ConstantsBall.kPIDLoopIdx, ConstantsBall.kGainsBall.kP, ConstantsBall.kTimeoutMs);
    _collectorRotateMotor.config_kI(ConstantsBall.kPIDLoopIdx, ConstantsBall.kGainsBall.kI, ConstantsBall.kTimeoutMs);
    _collectorRotateMotor.config_kD(ConstantsBall.kPIDLoopIdx, ConstantsBall.kGainsBall.kD, ConstantsBall.kTimeoutMs);
   
    /* Our profile uses 10ms timing */
    _collectorRotateMotor.configMotionProfileTrajectoryPeriod(10, ConstantsBall.kTimeoutMs);

    /*
     * Status 10 provides the trajectory target for motion profile AND motion magic
     */
    _collectorRotateMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ConstantsBall.kTimeoutMs);

    //resetEncoderPosition(0);
  }

  public void resetEncoderPosition(int position) {
    _collectorRotateMotor.setSelectedSensorPosition(position);
  }


  public boolean isMotionProfileFinished() {
    return _example.isMotionProfileDone();
  }

  public MotionProfileBallDouble getMP() {
    return _example;
  }

  public void setDirection(CollectorDirection direction) {
    if (direction == CollectorDirection.UP) {
       _collectorRotateMotor.setInverted(true);
    } else {
      _collectorRotateMotor.setInverted(false);   //NEEDS TO BE VERIFIED
    }

  }

}
