/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.HoustonClimber.ClimbLevel;
import frc.robot.subsystems.HoustonClimber.PodAction;


public class HoustonManualLevel3GroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HoustonManualLevel3GroupClimb() {
    //ROBOT UP
    double RAISEUP = 5.5;
    addParallel(new CloseBeak());
    addSequential(new HoustonManualClimb(PodAction.CLIMB, ClimbLevel.LEVEL3, RAISEUP));
    
    //HOLD and MOVE FORWARD TO CATCH the FIRST PART OF PLATFORM
    double HOLDANDMOVE = 2.4;
    double HOLDPOWER1 = .25;
    double HOLDPOWER2 = .2;

    addParallel(new DriveClimberWheels(0.03, 1.73, HOLDANDMOVE));
    addSequential(new HoustonClimberManualHold(HOLDPOWER1, HOLDPOWER2, 1, HOLDANDMOVE));
        
    double DRIVEFORWARD = 2.0;
    addParallel(new HoustonManualClimb(PodAction.RETRACT, ClimbLevel.LEVEL3, DRIVEFORWARD));
    addParallel(new DriveClimberWheels(1.0, DRIVEFORWARD, DRIVEFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, DRIVEFORWARD));
    
    double FINALFORWARD = 30.0;
    addParallel(new DriveClimberWheels(0.03, FINALFORWARD, FINALFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, FINALFORWARD));
  }
}



    /*
    double RAMPINGSTEPSIZE = 0.03;
    double STEPSPERSECOND = 100;
    double HERTZ
    double CRUISETIME = HOLDANDMOVE - ((RAMPINGSTEPSIZE * STEPSPERSECOND) * );
    
    number of steps is 1 divided by stepsize
    100/RAMPRATE = 33.333333333
    50 steps in a second
    33.33333 is 2/3 of 50

    50 steps a second1
    100 times .03 steps (steps = 100 * .03)

    takes 33 steps * .02 seconds
    */