package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
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
  public static final int[] ENC_DIO_ELEVATOR = {0,1};
  public static final boolean ENC_INVERT_COUNT_FALSE = false;
  public static final boolean ENC_INVERT_COUNT_TRUE = true;
  public static final int DISTANCE_PER_PULSE = 200; // TODO: Revisit this value
  public static final double MAX_PERIOD = 0.1; // TODO: Revisit this value
  public static final int MIN_RATE = 10; // TODO: Revisit this value
  public static final int SAMPLES_TO_AVERAGE = 7; // TODO: Revisit this value

  public CANPIDController _elevatorCANPID = null;

  private double _elev_kp = 1.0;
  private double _elev_ki = 0.0;
  private double _elev_kd = 0.0;
  private double _elev_max_output = 1.0;
  private double _elev_min_output = -1.0;
  private double _governor = 0.25;




  // Subsystem Constructor
  public Elevator() {
    // Define Subsystem Hardware
    _elevator = new CANSparkMax(RobotMap.elevatorMotorPort, MotorType.kBrushless);
    _enc_elevator = new CANEncoder(_elevator);
    //SetupEncoder(_enc_elevator,  "ELEVATOR", false);

    _elevatorCANPID = new CANPIDController(_elevator);
    // None of these commands will persist unless burnFlash() is called
    _elevatorCANPID.setP(_elev_kp);
    _elevatorCANPID.setI(_elev_ki);
    _elevatorCANPID.setD(_elev_kd);
    _elevatorCANPID.setOutputRange(_elev_min_output, _elev_max_output);
  }

  // Method to define the default command for the Elevator
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveElevator());
  }

  // Method to setup an encoder
  /*private void SetupEncoder(CANEncoder enc, String name, boolean reverseDirection) {
    enc.setName(name);
    System.out.println("Encoder: " + enc.getName());
    enc.setMaxPeriod(MAX_PERIOD);
    enc.setMinRate(MIN_RATE);
    System.out.println("Distance per Pulse: " + DISTANCE_PER_PULSE);
    enc.setDistancePerPulse(DISTANCE_PER_PULSE);
    enc.setReverseDirection(reverseDirection);
    enc.setSamplesToAverage(SAMPLES_TO_AVERAGE);
    enc.reset();
  }*/

  // Method that returns the current Elevator height
  public double getElevatorHeight() {
    if (_enc_elevator != null) {
      return _enc_elevator.getPosition();
    } else {
      return 0;
    }
  }

  // Method that resets the Elevator encoder
  /*public void resetElevatorEncoder() {
    _enc_elevator.reset();
  }*/

  // Method to move the Elevator based off the joystickValue
  public void moveElevator(double joystickValue) {
    _elevator.set(joystickValue* _governor);
  }

  public double getElevKp() {
    return this._elev_kp;
  }

  public double getElevKi() {
    return this._elev_ki;
  }

  public double getElevKd() {
    return this._elev_kd;
  }


}