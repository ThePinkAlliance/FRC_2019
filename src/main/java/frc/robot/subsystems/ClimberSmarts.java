/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ClimberSmarts extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Declare Digital Input Devices for this Subsystem
  public DigitalInput _frontOpticalLimitSwitch = null;
  public DigitalInput _rearOpticalLimitSwitch = null;

  // Declare Motors for this Subsystem
  public WPI_VictorSPX _climberWheels = null;

  // Define Open and Closed Booleans TODO: Check These Values
  public boolean OPTICAL_SWITCH_IN_RANGE = true;
  public boolean OPTICAL_SWITCH_OUT_RANGE = false;

  public ClimberSmarts() {
    // Define DigitalInput Devices for this Subsystem
    _frontOpticalLimitSwitch = new DigitalInput(RobotMap.bellyPanFrontSwitchPort);
    _rearOpticalLimitSwitch = new DigitalInput(RobotMap.bellyPanRearSwitchPort);

    // Define Motors for this Subsystem
    _climberWheels = new WPI_VictorSPX(RobotMap.climberSmartsForwardRightMotor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // Return Current Right Optical Limit Switch State
  public boolean getFrontOpticalLimitSwitch() {
    return _frontOpticalLimitSwitch.get();
  }

  // Return Current Left Optical Limit Switch State
  public boolean getRearOpticalLimitSwitch() {
    return _rearOpticalLimitSwitch.get();
  }

  // Set Climber Wheels to a Power
  public void driveClimberWheels(double power) {
    _climberWheels.set(power);
  }

  // Returns if Front Optical Limit Sensors are In Range
  public boolean safeToRaiseFrontClimberPods() {
    if (getFrontOpticalLimitSwitch() == OPTICAL_SWITCH_IN_RANGE) {
      return OPTICAL_SWITCH_IN_RANGE;
    } else {
      return OPTICAL_SWITCH_OUT_RANGE;
    }
  }

  // Returns if Rear Optical Limit Sensors are In Range
  public boolean safeToRaiseRearClimberPods() {
    if (getRearOpticalLimitSwitch() == OPTICAL_SWITCH_IN_RANGE) {
      return OPTICAL_SWITCH_IN_RANGE;
    } else {
      return OPTICAL_SWITCH_OUT_RANGE;
    }
  }
}