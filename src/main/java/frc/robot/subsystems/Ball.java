package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Hold;

public class Ball extends Subsystem {
  public Spark _collectorMotor = null;
  public WPI_TalonSRX _collectorRotateMotor = null;
  public DigitalInput collectedOpticalSwitch = null;

  public static double rotateKd = 0.1;
  public static double rotateKp = 0.1;
  public double collectSpeed = -1;
  public double holdSpeed = -0.2;
  public double ejectSpeed = 1;

  public Ball() {
    _collectorMotor = new Spark(RobotMap.collectorMotorPort);
    _collectorRotateMotor = new WPI_TalonSRX(RobotMap.collectorRotateMotorPort);
    _collectorRotateMotor.setNeutralMode(NeutralMode.Brake);
    collectedOpticalSwitch = new DigitalInput(RobotMap.collectedOpticalSwitchPort);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand();
    setDefaultCommand(new Hold());
  }

  // Method to move the Collector based off the joystickValue
  public void moveBall(double joystickValue) {
    _collectorRotateMotor.set(joystickValue);
    System.out.println("Collector Rotate Value: "+ joystickValue);
  }

  public void collect() {
    _collectorMotor.set(collectSpeed);
    System.out.println("Collector Power: " + _collectorMotor.get());
  }

  public void hold() {
    _collectorMotor.set(holdSpeed);
  }

  public void eject() {
    _collectorMotor.set(ejectSpeed);
  }

  public boolean getCollectState() {
    return collectedOpticalSwitch.get();
  }
}
