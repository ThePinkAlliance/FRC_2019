package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.RaiseElevatorToPosition;
import frc.robot.commands.RaiseElevatorToPosition.RaiseToPosition;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.JoystickElevator;

// Subsystem used for defining Elevator hardware and methods
public class Elevator extends Subsystem {
  // Declare Subsystem Variables
  public CANSparkMax _elevator = null;
  public CANPIDController elevator_pidController;
  public CANEncoder _enc_elevator = null;
  public DigitalInput _elevatorTopSwitch = null;
  public DigitalInput _elevatorBottonSwitch = null;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double elevatorKp = 0.05;

  // Subsystem Constructor
  public Elevator() {
    // Define Subsystem Hardware
    _elevator = new CANSparkMax(RobotMap.elevatorMotorPort, MotorType.kBrushless);
    _enc_elevator = new CANEncoder(_elevator);
    // _elevatorTopSwitch = new DigitalInput(2);
    // _elevatorBottonSwitch = new DigitalInput(1);
    initMotor();
  }

  // Method to define the default command for the Elevator
  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new RaiseElevatorToPosition(RaiseToPosition.JOYSTICK));
    setDefaultCommand(new JoystickElevator());
  }

  public boolean getElevatorTopSwitch() {
    return _elevatorTopSwitch.get();
  }

  public boolean getElevatorBottomSwitch() {
    return _elevatorBottonSwitch.get();
  }

  // Method that returns the current Elevator height
  public double getElevatorHeight() {
    return _enc_elevator.getPosition();
  }

  // Method to move the Elevator based off the joystickValue
  public void moveElevator(double joystickValue) {
     //System.out.println("Setting Elevator Power to " + joystickValue);
    _elevator.set(joystickValue);
  }

  public void moveElevatorToPosition(double targetPosition) {
    double elevator_motor_command = elevatorKp * (targetPosition - _enc_elevator.getPosition());
    _elevator.set(elevator_motor_command);
  }

  public void initMotor() {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    _elevator.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    elevator_pidController = _elevator.getPIDController();
    _enc_elevator = _elevator.getEncoder();

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    elevator_pidController.setP(kP);
    elevator_pidController.setI(kI);
    elevator_pidController.setD(kD);
    elevator_pidController.setIZone(kIz);
    elevator_pidController.setFF(kFF);
    elevator_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    elevator_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    elevator_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    elevator_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    elevator_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }

  public void handleDashboardValues() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { elevator_pidController.setP(p); kP = p; }
    if((i != kI)) { elevator_pidController.setI(i); kI = i; }
    if((d != kD)) { elevator_pidController.setD(d); kD = d; }
    if((iz != kIz)) { elevator_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { elevator_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      elevator_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { elevator_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { elevator_pidController.setSmartMotionMaxVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { elevator_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { elevator_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allE = allowedErr; }
  }

  // Method to get encoder position
  public double getEncPosition() {
    return _elevator.getEncoder().getPosition();
  }

  // Method to set encoder position
  public void setEncoderPosition(int position) {
    _elevator.setEncPosition(0);
  }

  // Method to set speed
  public void set(double speed) {
    _elevator.set(speed);
  }

  // Method to stop motor
  public void stopMotor() {
    if (_elevator != null) {
       _elevator.stopMotor();
    }
  }

  // Method that returns PID Controller
  public CANPIDController getPIDController() {
    return elevator_pidController;
  }

  // Method that returns the encoder
  public CANEncoder getEncoder() {
    return _enc_elevator;
  }

  // Method that returns elevator motor
  public CANSparkMax getMotor() {
    return _elevator;
  }
}