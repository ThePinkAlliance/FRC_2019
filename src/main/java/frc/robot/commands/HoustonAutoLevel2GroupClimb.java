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
    double PREP = .2;
    addParallel(new CloseBeak());
    addSequential(new BallRotateToPosition(PresetPositions.BALL_CLIMB_POSITION, 0.0005, PREP, false));

    double RAISEUP = 2.0;
    addParallel(new DriveClimberWheelsMove(1, RAISEUP));
    addSequential(new HoustonAutoLevel2Climb(PodAction.CLIMB, 0.0015, PresetPositions.BALL_CLIMB_POSITION, RAISEUP));
    //addSequential(new DriveClimberWheelsMove(1, RAISEUP));
       
    double DRIVEFORWARD = 3.0;
    //retracting climber pods is manual, raise ball collector by set position rather than by motion profile
    addParallel(new HoustonManualClimb(PodAction.RETRACT, ClimbLevel.LEVEL2, DRIVEFORWARD));
    addParallel(new BallRotateToPosition(0, 0.0003, DRIVEFORWARD, false));
    addParallel(new DriveClimberWheelsMove(1, DRIVEFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, DRIVEFORWARD));
    
        
    double FINALFORWARD = 30.0;
    addParallel(new HoustonBallFindStowAgain());
    addParallel(new DriveClimberWheels(0.03, FINALFORWARD, FINALFORWARD ));
    addSequential(new HoustonClimberDriveTrain(0.3, FINALFORWARD));
    
  }
}
