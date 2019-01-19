package frc.pink_233;

import edu.wpi.first.wpilibj.Joystick;

// This class contains the global variables for the controller input
public class Controls {
  // Declare Joysticks
  public static Joystick base = new Joystick(0);

  // Declare Joystick Axises
  public static double base_left_stick_y = base.getRawAxis(1);
  public static double base_right_stick_y = base.getRawAxis(3);
}