/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;


public class MotionProfileGroupClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MotionProfileGroupClimb() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    //addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.UP, PodPosition.FRONT, .15, 4, 1));
    // addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.UP, PodPosition.FRONT, .15, 4, 1));
    // addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackLeft,   ClimberDirection.UP, PodPosition.BACK,  .15, 4, 1));
    // addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackRight,  ClimberDirection.UP, PodPosition.BACK,  .15, 4, 1));
    //addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.UP, PodPosition.FRONT, .15, 4, 1));
    addSequential(new MotionProfileClimberMasterTest(ClimberDirection.UP, .15, 0.5, 1));
    //addParallel(new HoldClimberPosition(8));
    //addSequential(new DriveClimberWheels());
  }
}
