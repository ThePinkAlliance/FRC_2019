/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Compressor _compressor = null;
  public Solenoid _beak = null;
  public Solenoid _neck = null;
  public boolean _beakOpen = false;
  public boolean _neckUp = false;

  public Hatch() {
    _compressor = new Compressor(0);
    _beak = new Solenoid(1);
    _neck = new Solenoid(2);
  }

  @Override
  public void initDefaultCommand() {
  }

  public void toggleBeak() {
    _beakOpen = !_beakOpen;
    _beak.set(_beakOpen);
  }

  public void toggleNeck() {
    _neckUp = !_neckUp;
    _neck.set(_neckUp);
  }
}
