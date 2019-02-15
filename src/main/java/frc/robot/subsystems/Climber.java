package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveClimber;

// Subsystem used for defineing Climber hardware and methods
public class Climber extends Subsystem {
  // Define Subsystem Variables
  public WPI_TalonSRX _climberMotor = null;
  public Encoder _enc_climber = null;
  public static final int[] ENC_DIO_CLIMBER = {0,1};
  public static final boolean ENC_INVERT_COUNT_FALSE = false;
  public static final boolean ENC_INVERT_COUNT_TRUE = true;
  public static final int DISTANCE_PER_PULSE = 200; // TODO: Revisit this value
  public static final double MAX_PERIOD = 0.1; // TODO: Revisit this value
  public static final int MIN_RATE = 10; // TODO: Revisit this value
  public static final int SAMPLES_TO_AVERAGE = 7; // TODO: Revisit this value

  // Subsystem Constuctor
  public Climber() {
    // Define Subsystem Hardware
    _climberMotor = new WPI_TalonSRX(RobotMap.climberMotorPort);
    _climberMotor.setNeutralMode(NeutralMode.Brake);
    //_enc_climber = new Encoder(ENC_DIO_CLIMBER[0], ENC_DIO_CLIMBER[1], ENC_INVERT_COUNT_FALSE, Encoder.EncodingType.k4X);
    //SetupEncoder(_enc_climber,  "CLIMBER", false);
  }

  // Method to define the default command for the Climber
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveClimber());
  }

  // Method to setup an encoder
  //  private void SetupEncoder(Encoder enc, String name, boolean reverseDirection) {
  //   enc.setName(name);
  //   System.out.println("Encoder: " + enc.getName());
  //   enc.setMaxPeriod(MAX_PERIOD);
  //   enc.setMinRate(MIN_RATE);
  //   System.out.println("Distance per Pulse: " + DISTANCE_PER_PULSE);
  //   enc.setDistancePerPulse(DISTANCE_PER_PULSE);
  //   enc.setReverseDirection(reverseDirection);
  //   enc.setSamplesToAverage(SAMPLES_TO_AVERAGE);
  //   enc.reset();
  // }

  // Method that returns the current Climber height
  public double getClimberHeight() {
    if (_enc_climber != null) {
      return _enc_climber.getDistance();
    } else {
      return 0;
    }
  }

  // Method that resets the Climber encoder
  public void resetClimberEncoder() {
    _enc_climber.reset();
  }

  // Method to move the Climber based off the joystickValue
  public void moveClimber(double joystickValue) {
    _climberMotor.set(joystickValue);
  }
}
