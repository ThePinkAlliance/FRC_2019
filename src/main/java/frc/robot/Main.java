package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// Do not modify this file except to change the parameter class to the startRobot call
public final class Main {

  // Construct Class
  private Main() {
  }

  // Do not perform any initialization here. If you change your main robot class, change the parameter type.
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}