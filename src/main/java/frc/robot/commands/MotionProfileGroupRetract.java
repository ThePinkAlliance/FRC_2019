/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class MotionProfileGroupRetract extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MotionProfileGroupRetract() {
    // double DRIVEFORWARD = 3.0;
    // addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontLeft, .3, .2, 1, DRIVEFORWARD));
    // addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontRight, .3, .2, 1, DRIVEFORWARD));
    // addParallel(new DriveClimberWheels(0.03, 2.0, DRIVEFORWARD ));
    // addSequential(new MotionProfileClimberDriveTrain(0.2, DRIVEFORWARD));
    
    
    // addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.DOWN, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, 5.5, 1));
    // addSequential(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.DOWN, PodPosition.FRONT, ClimbLevel.LEVEL3, .15, 5.5, 1));
    // addParallel(new DriveClimberWheels(0.03, 30.0, 31.0 ));
    // addSequential(new MotionProfileClimberDriveTrain(0.3, 31.0));

  }
}
