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
import frc.robot.subsystems.utils.PresetPositions;


public class HoustonAutoLevel2GroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HoustonAutoLevel2GroupClimb() {
    //ROBOT UP
    double PREPARE = 1.0;
    addParallel(new CloseBeak());
    addSequential(new BallRotateToPosition(PresetPositions.BALL_CLIMB_POSITION, 0.0005, PREPARE, false));


    double RAISEUP = 5.5;
    addSequential(new HoustonAutoLevel2Climb(PodAction.CLIMB, 0.0015, PresetPositions.BALL_CLIMB_POSITION, RAISEUP));
    
    //HOLD and MOVE FORWARD TO CATCH the FIRST PART OF PLATFORM
    double HOLDANDMOVE = 2.4;
    double HOLDPOWER1 = .25;
    double HOLDPOWER2 = .2;
    addParallel(new DriveClimberWheels(0.03, 1.73, HOLDANDMOVE));
    addSequential(new HoustonClimberAutoHold(HOLDPOWER1, HOLDPOWER2, 1, HOLDANDMOVE));
       
    double DRIVEFORWARD = 3.0;
    //retracting climber pods is manual, raise ball collector by set position rather than by motion profile
    addParallel(new HoustonManualClimb(PodAction.RETRACT, ClimbLevel.LEVEL2, DRIVEFORWARD));
    addParallel(new BallRotateToPosition(0, 0.0003, DRIVEFORWARD, false));
    addParallel(new DriveClimberWheels(1.0, 2.37, DRIVEFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, DRIVEFORWARD));
    
        
    double FINALFORWARD = 30.0;
    addParallel(new HoustonBallFindStowAgain());
    addParallel(new DriveClimberWheels(0.03, FINALFORWARD, FINALFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, FINALFORWARD));
    
  }
}
