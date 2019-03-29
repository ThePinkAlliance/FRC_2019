/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.utils.PresetPositions;
import frc.robot.subsystems.utils.MotionProfileBallDouble.CollectorDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimbLevel;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;


public class MotionProfileLevel3GroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MotionProfileLevel3GroupClimb(boolean manual_override) {
    //ROBOT UP
    // addParallel(new ToggleNeck());
    addSequential(new BallRotateToPosition(PresetPositions.BALL_CLIMB_LEVEL3_MOVE1, 0.0003, 1, manual_override));

    double RAISEUP = 5.5;
    addParallel(new MotionProfileClimberTestDoubleLevel3(Robot.m_climberPodFrontLeft, ClimberDirection.UP, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, RAISEUP, 0.0015, PresetPositions.BALL_CLIMB_POSITION, manual_override, CollectorDirection.UP));
    addSequential(new MotionProfileClimberTestDoubleLevel3(Robot.m_climberPodFrontRight, ClimberDirection.UP, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, RAISEUP, 0.0015, PresetPositions.BALL_CLIMB_POSITION, manual_override, CollectorDirection.UP));
    
    double DRIVEFORWARD = 3.0;
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontLeft, .25, .25, 1, DRIVEFORWARD, manual_override));
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontRight, .25, .25, 1, DRIVEFORWARD, manual_override));
    addParallel(new DriveClimberWheels(0.02, 2.0, DRIVEFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.25, DRIVEFORWARD));
    
    double RETRACTDRIVEFORWARD = 2.5;
    addParallel(new MotionProfileClimberTestDoubleLevel3(Robot.m_climberPodFrontLeft,  ClimberDirection.DOWN, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, RETRACTDRIVEFORWARD, 0.0005, PresetPositions.BALL_CARGO_POSITION, manual_override, CollectorDirection.DOWN));
    addParallel(new MotionProfileClimberTestDoubleLevel3(Robot.m_climberPodFrontRight, ClimberDirection.DOWN, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, RETRACTDRIVEFORWARD, 0.0005, PresetPositions.BALL_CARGO_POSITION, manual_override, CollectorDirection.DOWN));
    addParallel(new DriveClimberWheels(1, DRIVEFORWARD, DRIVEFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.25, DRIVEFORWARD));

    double HITALLIANCEWALL = 30;
    //addParallel(new BallFindStowAgain(-0.15, 1));  //DOES NOT WORK, do not uncomment, here only for reference point
    addParallel(new DriveClimberWheels(0.03, HITALLIANCEWALL, HITALLIANCEWALL ));
    addSequential(new MotionProfileClimberDriveTrain(0.25, HITALLIANCEWALL));
    
    
  }
}
