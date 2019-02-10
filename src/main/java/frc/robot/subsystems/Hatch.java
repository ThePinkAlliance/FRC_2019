package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

// Subsystem used for defining Hatch hardware and methods
public class Hatch extends Subsystem {
  // Declare Subsystem Variables
  public Solenoid _beak = null;
  public Solenoid _neck = null;
  public boolean _beakOpen = false;
  public boolean _neckUp = false;

  // Subsystem Constructor
  public Hatch() {
    // Define Subsystem Hardware
    _beak = new Solenoid(RobotMap.beakPort);
    _neck = new Solenoid(RobotMap.neckPort);
  }

  // Method to define the default command for the Hatch
  @Override
  public void initDefaultCommand() {
  }

  // Method to toggle the Beak solenoid on the Hatch
  public void toggleBeak() {
    _beakOpen = !_beakOpen;
    _beak.set(_beakOpen);
  }

  // Method to toggle the Neck solenoid on the Hatch
  public void toggleNeck() {
    _neckUp = !_neckUp;
    _neck.set(_neckUp);
  }
}