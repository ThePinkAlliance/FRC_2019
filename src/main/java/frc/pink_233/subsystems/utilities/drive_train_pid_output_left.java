/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.pink_233.subsystems.utilities;

import edu.wpi.first.wpilibj.PIDOutput;
import frc.pink_233.robot;

/**
 * Right side pid output implementation.
 */
public class drive_train_pid_output_left implements PIDOutput {

    public drive_train_pid_output_left() {

    }

    @Override
    public void pidWrite(double output) {
       robot.drive_train.leftMotor(output);
    }
}