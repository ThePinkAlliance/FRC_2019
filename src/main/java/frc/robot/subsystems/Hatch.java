package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ToggleBeakByLimitSwitch;

// Subsystem used for defining Hatch hardware and methods
public class Hatch extends Subsystem {
  // Declare Subsystem Variables
  public Solenoid _beak = null;
  public Solenoid _neck = null;
  public DigitalInput limitSwitchHatchCollected = null;
  public boolean _beakOpen = true;
  //public boolean _beakToggle = false; // not being used
  public boolean _neckUp = false;
  public boolean _manualToggle = false;

  // Subsystem Constructor
  public Hatch() {
    // Define Subsystem Hardware
    _beak = new Solenoid(RobotMap.beakPort);
    _neck = new Solenoid(RobotMap.neckPort);
    limitSwitchHatchCollected = new DigitalInput(RobotMap.limitSwitchHatchCollectedPort);
  }

  // Method to define the default command for the Hatch
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ToggleBeakByLimitSwitch());
  }

  /*
  //NOT BEING USED
  // Method to toggle the Beak solenoid on the Hatch
  public void toggleBeak() {
    _beakToggle = !_beakToggle;
    _beak.set(_beakToggle);
  }
  */

  // Method to toggle the Neck solenoid on the Hatch
  public void toggleNeck() {
    _neckUp = !_neckUp;
    _neck.set(_neckUp);
  }
}