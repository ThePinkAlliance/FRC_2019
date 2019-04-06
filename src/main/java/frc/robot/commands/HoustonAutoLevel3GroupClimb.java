/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.HoustonClimber.PodAction;
import frc.robot.subsystems.utils.PresetPositions;


public class HoustonAutoLevel3GroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HoustonAutoLevel3GroupClimb() {
    //ROBOT UP
    double PREPARE = 1.0;
    addParallel(new CloseBeak());
    addSequential(new BallRotateToPosition(PresetPositions.BALL_CLIMB_LEVEL3_MOVE1, 0.0003, PREPARE, false));

    double RAISEUP = 5.5;
    addSequential(new HoustonAutoLevel3Climb(PodAction.CLIMB, RAISEUP));
    
    //HOLD and MOVE FORWARD TO CATCH the FIRST PART OF PLATFORM
    double HOLDANDMOVE = 2.4;
    double HOLDPOWER1 = .25;
    double HOLDPOWER2 = .2;
    addParallel(new DriveClimberWheels(0.03, 1.73, HOLDANDMOVE));
    addSequential(new HoustonClimberAutoHold(HOLDPOWER1, HOLDPOWER2, 1, HOLDANDMOVE));
       
    double DRIVEFORWARD = 3.0;
    //retracting climber pods is manual, raise ball collector by set position rather than by motion profile
    addParallel(new HoustonManualLevel3Climb(PodAction.RETRACT, DRIVEFORWARD));
    addParallel(new BallRotateToPosition(0, 0.0003, DRIVEFORWARD, false));
    addParallel(new DriveClimberWheels(1.0, 2.37, DRIVEFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, DRIVEFORWARD));
    
    double ALMOSTFINALFORWARD = 1;
    addParallel(new BallRotateToPosition(-(PresetPositions.BALL_CLIMB_LEVEL3_MOVE1), 0.0003, ALMOSTFINALFORWARD, false));
    addParallel(new DriveClimberWheels(1.0, ALMOSTFINALFORWARD, ALMOSTFINALFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, ALMOSTFINALFORWARD));
    
    double FINALFORWARD = 30.0;
    addParallel(new HoustonBallFindStowAgain());
    addParallel(new DriveClimberWheels(0.03, FINALFORWARD, FINALFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, FINALFORWARD));
    
  }
}
