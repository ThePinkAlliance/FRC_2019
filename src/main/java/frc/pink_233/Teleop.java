package frc.pink_233;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.pink_233.Subsystems.drive_train;
import frc.pink_233.Controls;

// This class contains the code which will run when driving the robot in teleop
public class Teleop extends TimedRobot {
  // Declare Class Variables
  private DifferentialDrive robot;

  // This code will run once upon initialization
  @Override
  public void robotInit() {
    // Establish a Differential Drive with the motors established in Subsystems.drive_train
    robot = new DifferentialDrive(drive_train.left_motors, drive_train.right_motors);
  }

  // This code will run repeatedly during teleop
  @Override
  public void teleopPeriodic() {
    // Start Tank Drive using the left and right joysticks on the base (Port 0) controller
    robot.tankDrive(Controls.base_left_stick_y, Controls.base_right_stick_y);
  }
}
