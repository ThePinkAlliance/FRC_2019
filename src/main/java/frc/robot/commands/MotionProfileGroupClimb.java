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

    //ROBOT UP
    double RAISEUP = 5.5;
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.UP, PodPosition.FRONT, .15, RAISEUP, 1));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.UP, PodPosition.FRONT, .15, RAISEUP, 1));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackLeft,   ClimberDirection.UP, PodPosition.BACK,  .15, RAISEUP, 1));
    addSequential(new MotionProfileClimberTestDouble(Robot.m_climberPodBackRight,  ClimberDirection.UP, PodPosition.BACK,  .15, RAISEUP, 1));
    
    //HOLD and MOVE FORWARD TO CATCH the FIRST PART OF PLATFORM
    double HOLDANDMOVE = 2.4;
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontLeft, .3, .2, 1, HOLDANDMOVE));
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontRight, .3, .2, 1, HOLDANDMOVE));
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodBackLeft, .3, .2, 1, HOLDANDMOVE));
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodBackRight, .3, .2, 1, HOLDANDMOVE));
    addSequential(new DriveClimberWheels(0.03, 0.6, HOLDANDMOVE));
    
    //RETRACT FIRST SET OF PODS (back pods)
    double RETRACTFIRSTSET = 3.1;
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontLeft, .3, .2, 1, RETRACTFIRSTSET));
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontRight, .3, .2, 1, RETRACTFIRSTSET));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackLeft,   ClimberDirection.DOWN, PodPosition.BACK,  .15, RETRACTFIRSTSET, 1));
    addSequential(new MotionProfileClimberTestDouble(Robot.m_climberPodBackRight,  ClimberDirection.DOWN, PodPosition.BACK,  .15, RETRACTFIRSTSET, 1));
    
    double DRIVEFORWARD = 3.0;
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontLeft, .3, .2, 1, DRIVEFORWARD));
    addParallel(new MotionProfileClimberHoldByPower(Robot.m_climberPodFrontRight, .3, .2, 1, DRIVEFORWARD));
    addParallel(new DriveClimberWheels(0.03, 2.0, DRIVEFORWARD ));
    addSequential(new MotionProfileClimberDriveTrain(0.2, DRIVEFORWARD));
    
    
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.DOWN, PodPosition.FRONT, .15, 5.5, 1));
    addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.DOWN, PodPosition.FRONT, .15, 5.5, 1));
    addParallel(new DriveClimberWheels(0.03, 30.0, 31.0 ));
    addSequential(new MotionProfileClimberDriveTrain(0.3, 31.0));
    
    // addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackLeft,   ClimberDirection.DOWN, PodPosition.BACK,  .15, 5.5, 1));
    // addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodBackRight,  ClimberDirection.DOWN, PodPosition.BACK,  .15, 5.5, 1));
    // addParallel(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontLeft,  ClimberDirection.DOWN, PodPosition.FRONT, .15, 5.5, 1));
    // addSequential(new MotionProfileClimberTestDouble(Robot.m_climberPodFrontRight, ClimberDirection.DOWN, PodPosition.FRONT, .15, 5.5, 1));
    
    
  }
}
