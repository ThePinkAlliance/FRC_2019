package frc.pink_233;

import edu.wpi.first.wpilibj.RobotBase;

// Do NOT add any static variables to this class, or any initialization at all.
public final class Main {
  private Main() {
  }

  //Main initialization function. Do not perform any initialization here.
  public static void main(String... args) {
    RobotBase.startRobot(Teleop::new);
  }
}
