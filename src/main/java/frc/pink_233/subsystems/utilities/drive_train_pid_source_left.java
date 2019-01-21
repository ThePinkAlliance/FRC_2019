/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.pink_233.subsystems.utilities;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.pink_233.robot;

/**
 * Add your docs here.
 */
public class drive_train_pid_source_left implements PIDSource {

    private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;

    @Override 
  public double pidGet() {
     //Get the amount left to target
     return robot.drive_train.getFrontRightDistance();
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSourceType) {
    this.pidSourceType = pidSourceType;
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return pidSourceType;
  }


}