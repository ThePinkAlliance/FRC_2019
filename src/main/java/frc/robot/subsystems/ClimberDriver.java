package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ClimberDriver extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  // Declare Motors for this Subsystem
  public WPI_VictorSPX _climberWheelL = null;
  public WPI_TalonSRX _climberWheelR = null;

  public ClimberDriver() {

    // Define Motors for this Subsystem
    _climberWheelL = new WPI_VictorSPX(RobotMap.climberDriverForwardLeftMotor);
    _climberWheelR = new WPI_TalonSRX(RobotMap.climberDriverForwardRightMotor);

    _climberWheelL.setInverted(true);
    _climberWheelR.setInverted(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // Set Climber Wheels to a Power
  public void setClimberWheels(double power) {
    _climberWheelL.set(ControlMode.PercentOutput, power);
    _climberWheelR.set(ControlMode.PercentOutput, power);
  }

}