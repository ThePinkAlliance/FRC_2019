package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveElevator;

// Subsystem used for defining Elevator hardware and methods
public class Elevator extends Subsystem {
  // Declare Subsystem Variables
  public CANSparkMax _elevator = null;
  public CANEncoder _enc_elevator = null;
  public static final int DISTANCE_PER_PULSE = 200; // TODO: Revisit this value

  private double _governor = 1;

  private double encoderBottom = 10;

  public double getEncoderBottom() {
    return encoderBottom;
  }

  // Subsystem Constructor
  public Elevator() {
    // Define Subsystem Hardware
    _elevator = new CANSparkMax(RobotMap.elevatorMotorPort, MotorType.kBrushless);
    _enc_elevator = new CANEncoder(_elevator);
    setBottomHeight();
  }

  // Method to define the default command for the Elevator
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveElevator());
  }

  // Method that returns the current Elevator height
  public double getElevatorHeight() {
    return _enc_elevator.getPosition();
  }

  // To be called in the constructor only, sets the bottom for duration of match
  public void setBottomHeight() {
    _enc_elevator.setPosition(0);
    encoderBottom = 0;
  }

  // Method to move the Elevator based off the joystickValue
  public void moveElevator(double joystickValue) {
    //System.out.println("Setting Elevator Power to " + joystickValue);
    _elevator.set(joystickValue);
  }
}