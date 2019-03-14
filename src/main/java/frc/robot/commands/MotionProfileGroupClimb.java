/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;


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

    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.UP, PodPosition.FRONT, .15, 4, 1));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.UP, PodPosition.FRONT, .15, 4, 1));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackLeft,   ClimberDirection.UP, PodPosition.BACK,  .15, 4, 1));
    addSequential(new MotionProfileClimberTestDouble(Robot.m_climberPodBackRight,  ClimberDirection.UP, PodPosition.BACK,  .15, 4, 1));
    // addParallel(new MotionProfileClimberHold(Robot.m_climberPodFrontLeft, 8));  
    // addParallel(new MotionProfileClimberHold(Robot.m_climberPodFrontRight, 8));
    // addParallel(new MotionProfileClimberHold(Robot.m_climberPodBackLeft, 8));
    // addParallel(new MotionProfileClimberHold(Robot.m_climberPodBackRight, 8));
    // addParallel(new MotionProfileClimberDriveTrain(.18, 6.0));
    // addParallel(new DriveClimberWheels(0.3, 0.25, 3));
    // addSequential(new MotionProfileClimberDriveTrain(0.2, 5.0));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackLeft,   ClimberDirection.DOWN, PodPosition.BACK,  .15, 4, 1));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackRight,  ClimberDirection.DOWN, PodPosition.BACK,  .15, 4, 1));
    // addSequential(new DriveClimberWheels(0.3, 1, 3.5));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.DOWN, PodPosition.FRONT, .15, 4, 1));
    addSequential(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.DOWN, PodPosition.FRONT, .15, 4, 1));
    
    //addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.UP, PodPosition.FRONT, .15, 4, 1));
    //addSequential(new MotionProfileClimberMasterTest(ClimberDirection.UP, .15, 1.5, 1));
    //addParallel(new HoldClimberPosition(8));
    //addSequential(new DriveClimberWheels());
  }
}
