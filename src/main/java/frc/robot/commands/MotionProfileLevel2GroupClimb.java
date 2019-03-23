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
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimbLevel;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;


public class MotionProfileLevel2GroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MotionProfileLevel2GroupClimb(boolean manual_override) {
    //ROBOT UP
    double SETUP = 0.7;
    addParallel(new CloseBeak());
    addSequential(new BallRotateToPosition(PresetPositions.BALL_CLIMB_POSITION, 0.0005, SETUP, manual_override));

    double RAISEUP = 3;
    addParallel(new DriveClimberWheelsMove(1, 2.4));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.UP, PodPosition.FRONT, ClimbLevel.LEVEL2, .15, RAISEUP, 0.0015, PresetPositions.BALL_CLIMB_POSITION, manual_override));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.UP, PodPosition.FRONT, ClimbLevel.LEVEL2, .15, RAISEUP, 0.0015, PresetPositions.BALL_CLIMB_POSITION, manual_override));
    addSequential(new MotionProfileClimberDriveTrain(0.3, 2.4));
   
    // double DRIVEFORWARD = 0.2;
    // addParallel(new MotionProfileClimberHold(DRIVEFORWARD, 0.0015));
    // addParallel(new DriveClimberWheelsMove(1, DRIVEFORWARD ));
    // addSequential(new MotionProfileClimberDriveTrain(0.3, DRIVEFORWARD));
    
    double RETRACT = 1.5;
    double DRIVEFINAL = 30;
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.DOWN, PodPosition.FRONT, ClimbLevel.LEVEL2, .15, RETRACT, 0.0005, PresetPositions.BALL_CARGO_POSITION, manual_override));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.DOWN, PodPosition.FRONT, ClimbLevel.LEVEL2, .15, RETRACT, 0.0005, PresetPositions.BALL_CARGO_POSITION, manual_override));
    addParallel(new DriveClimberWheelsMove(1, DRIVEFINAL));
    addSequential(new MotionProfileClimberDriveTrain(0.3, DRIVEFINAL));
  }
}
