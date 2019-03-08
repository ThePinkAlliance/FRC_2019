package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Hold;

public class Ball extends Subsystem {

  // Init motor controllers
  public Spark _collectorMotor = null;
  public WPI_TalonSRX _collectorRotateMotor = null;
  public double _enc_collectorRotate = 0.0;

  // Init digital inputs
  public DigitalInput collectedOpticalSwitch = null;

  // Setup values
  // public static double rotateKd = 0.005;
  public static double rotateKp = 0.01;
  public double collectSpeed = -1;
  public double holdSpeed = -0.2;
  public double ejectSpeed = 1;
  public double COLLECT_POS = 0;
  public double LOW_ROCKET_POS = 100;
  public double CARGO_POS = 200;

  public Ball() {

    // Construct motor controllers
    _collectorMotor = new Spark(RobotMap.collectorMotorPort);
    _collectorRotateMotor = new WPI_TalonSRX(RobotMap.collectorRotateMotorPort);
    _collectorRotateMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    _enc_collectorRotate = _collectorRotateMotor.getSelectedSensorPosition();
    _collectorRotateMotor.setNeutralMode(NeutralMode.Brake);

    // Construct digital inputs
    collectedOpticalSwitch = new DigitalInput(RobotMap.collectedOpticalSwitchPort);
  }

  @Override
  public void initDefaultCommand() {
    // Set default command to be Hold
    setDefaultCommand(new Hold());
  }

  // Method to move the Collector based off the joystickValue
  public void moveBall(double joystickValue) {
    _collectorRotateMotor.set(joystickValue);
    //system..out.println("Collector Rotate Value: "+ joystickValue);
  }

  public void setRotateMotorCmd(double target_position) {
    double rotate_motor_command = rotateKp * (target_position - _enc_collectorRotate);
    _collectorRotateMotor.set(rotate_motor_command);
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
}
