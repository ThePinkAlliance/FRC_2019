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


public class MotionProfileGroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MotionProfileGroupClimb() {
    //ROBOT UP
    double RAISEUP = 5.5;
    addParallel(new CloseBeak());
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.UP, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, RAISEUP, 0.0015, PresetPositions.BALL_CLIMB_POSITION, false));
    addSequential(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.UP, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, RAISEUP, 0.0015, PresetPositions.BALL_CLIMB_POSITION, false));
    
    //HOLD and MOVE FORWARD TO CATCH the FIRST PART OF PLATFORM
    double HOLDANDMOVE = 2.4;
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontLeft, .25, .2, 1, HOLDANDMOVE));
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontRight, .25, .2, 1, HOLDANDMOVE));
    addSequential(new DriveClimberWheels(0.03, 0.6, HOLDANDMOVE));
   
    double DRIVEFORWARD = 3.0;
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontLeft, .25, .2, 1, DRIVEFORWARD));
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontRight, .25, .2, 1, DRIVEFORWARD));
    addParallel(new DriveClimberWheels(0.03, 2.0, DRIVEFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, DRIVEFORWARD));
    
    
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.DOWN, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, 5.5, 0.0005, PresetPositions.BALL_CARGO_POSITION, false));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.DOWN, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, 5.5, 0.0005, PresetPositions.BALL_CARGO_POSITION, false));
    addParallel(new DriveClimberWheels(0.03, 30.0, 31.0 ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, 31.0));
    
    
  }
}
