package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveCollector;
import frc.robot.commands.MoveCollector.CollectorTargetPosition;

public class Ball extends Subsystem {
  public WPI_TalonSRX _collectorMotor = null;
  public CANSparkMax _collectorRotateMotor = null;
  public CANEncoder _enc_collectorRotate = null;
  public static double rotateKd = 0.1;
  public static double rotateKp = 0.1;

  public Ball() {
    // _collectorMotor = new WPI_TalonSRX(RobotMap.collectorMotorPort);
    // _collectorRotateMotor = new CANSparkMax(RobotMap.collectorRotateMotorPort, MotorType.kBrushless);
    _enc_collectorRotate = new CANEncoder(_collectorRotateMotor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MoveCollector(CollectorTargetPosition.JOYSTICK));
  }

  public double getCurrentPosition() {
    double currentPosition = _enc_collectorRotate.getPosition();
    return currentPosition;
  }

  // Method to move the Collector based off the joystickValue
  public void moveBall(double joystickValue) {
    _collectorRotateMotor.set(joystickValue);
  }

  public void rotateToPosition(double position, double starting_position) {
    double rotateMotorCommand = (rotateKp * (starting_position - position) +
    (rotateKd * _enc_collectorRotate.getVelocity()));
    _collectorRotateMotor.set(rotateMotorCommand);
  }
}
