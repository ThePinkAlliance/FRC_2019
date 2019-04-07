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


public class HoustonAutoLevel3GroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HoustonAutoLevel3GroupClimb() {
    //ROBOT UP
    double PREP = 1.0;
    
    addParallel(new CloseBeak());
    addSequential(new BallRotateToPosition(PresetPositions.BALL_CLIMB_LEVEL3_MOVE1, 0.0003, PREP, false));
    
    double RAISEUP = 5.5;
    addSequential(new HoustonAutoLevel3Climb(PodAction.CLIMB, RAISEUP));
    
    //HOLD and MOVE FORWARD TO CATCH the FIRST PART OF PLATFORM
    double HOLDANDMOVE = 2.4;
    double HOLDPOWER1 = .15;
    double HOLDPOWER2 = .15;
    addParallel(new DriveClimberWheelsMove(1.0, HOLDANDMOVE));
    addSequential(new HoustonClimberAutoHold(HOLDPOWER1, HOLDPOWER2, 1, HOLDANDMOVE));
       
    double DRIVEFORWARD = 3.0;
    //retracting climber pods is manual, raise ball collector by set position rather than by motion profile
    addParallel(new HoustonManualClimb(PodAction.RETRACT, ClimbLevel.LEVEL3, DRIVEFORWARD));
    addParallel(new BallRotateToPosition(0, 0.0003, DRIVEFORWARD, false));
    addParallel(new DriveClimberWheelsMove(1.0, DRIVEFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, DRIVEFORWARD));
    
    double ALMOSTFINALFORWARD = 1;
    addParallel(new BallRotateToPosition(0, 0.0003, ALMOSTFINALFORWARD, false));
    addParallel(new DriveClimberWheelsMove(1.0, ALMOSTFINALFORWARD ));
    //addParallel(new DriveClimberWheels(1.0, ALMOSTFINALFORWARD, ALMOSTFINALFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, ALMOSTFINALFORWARD));
    
    double FINALFORWARD = 30.0;
   // addParallel(new HoustonBallFindStowAgain());
    addParallel(new DriveClimberWheelsMove(1, FINALFORWARD ));
    //addParallel(new DriveClimberWheels(0.03, FINALFORWARD, FINALFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, FINALFORWARD));
    
  }
}
