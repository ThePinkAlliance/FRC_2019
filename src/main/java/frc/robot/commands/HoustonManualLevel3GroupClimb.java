/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.HoustonClimber.PodAction;


public class HoustonManualLevel3GroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HoustonManualLevel3GroupClimb() {
    //ROBOT UP
    double RAISEUP = 5.5;
    addParallel(new CloseBeak());
    addSequential(new HoustonManualLevel3Climb(PodAction.CLIMB, RAISEUP));
    
    //HOLD and MOVE FORWARD TO CATCH the FIRST PART OF PLATFORM
    double HOLDANDMOVE = 2.4;
    addParallel(new HoustonClimberManualHold(.25, .2, 1, HOLDANDMOVE));
    addParallel(new HoustonClimberManualHold(.25, .2, 1, HOLDANDMOVE));
    addSequential(new DriveClimberWheels(0.03, 0.6, HOLDANDMOVE));
   
    double DRIVEFORWARD = 3.0;
    addParallel(new HoustonClimberManualHold(.25, .2, 1, DRIVEFORWARD));
    addParallel(new HoustonClimberManualHold(.25, .2, 1, DRIVEFORWARD));
    addParallel(new DriveClimberWheels(0.03, 2.0, DRIVEFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, DRIVEFORWARD));
    
    double FINALFORWARD = 5.5;
    addParallel(new HoustonManualLevel3Climb(PodAction.RETRACT, RAISEUP));
    addParallel(new DriveClimberWheels(0.03, 30.0, 31.0 ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, 31.0));
    
    
  }
}
