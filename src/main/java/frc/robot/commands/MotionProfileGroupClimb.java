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
import frc.robot.commands.MotionProfileClimberManual3;


public class MotionProfileGroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MotionProfileGroupClimb() {
    //ROBOT UP
    double RAISEUP = 5.5;
    addParallel(new CloseBeak());
    addParallel(new MotionProfileClimberManual3(Robot.m_climberPodFrontLeft,  ClimberDirection.UP, PodPosition.FRONT, .15, RAISEUP, 1));
    addSequential(new MotionProfileClimberManual3(Robot.m_climberPodFrontRight, ClimberDirection.UP, PodPosition.FRONT, .15, RAISEUP, 1));
    
    //HOLD and MOVE FORWARD TO CATCH the FIRST PART OF PLATFORM
    double HOLDANDMOVE = 2.4;
    addParallel(new MotionProfileClimberHoldManual(Robot.m_climberPodFrontLeft, .25, .2, 1, HOLDANDMOVE));
    addParallel(new MotionProfileClimberHoldManual(Robot.m_climberPodFrontRight, .25, .2, 1, HOLDANDMOVE));
    addSequential(new DriveClimberWheels(0.03, 0.6, HOLDANDMOVE));
   
    double DRIVEFORWARD = 3.0;
    addParallel(new MotionProfileClimberHoldManual(Robot.m_climberPodFrontLeft, .25, .2, 1, DRIVEFORWARD));
    addParallel(new MotionProfileClimberHoldManual(Robot.m_climberPodFrontRight, .25, .2, 1, DRIVEFORWARD));
    addParallel(new DriveClimberWheels(0.03, 2.0, DRIVEFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, DRIVEFORWARD));
    
    
    addParallel(new MotionProfileClimberManual3(Robot.m_climberPodFrontLeft,  ClimberDirection.DOWN, PodPosition.FRONT, .15, 5.5, 1));
    addParallel(new MotionProfileClimberManual3(Robot.m_climberPodFrontRight, ClimberDirection.DOWN, PodPosition.FRONT, .15, 5.5, 1));
    addParallel(new DriveClimberWheels(0.03, 30.0, 31.0 ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, 31.0));
    
    
  }
}
