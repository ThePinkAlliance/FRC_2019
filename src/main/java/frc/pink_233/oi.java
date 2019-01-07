package frc.pink_233;

import edu.wpi.first.wpilibj.Joystick;


public class oi {
	public Joystick base = new Joystick(robot_map.baseJoystickPort);
	
	public oi() {
	}
	
	public Joystick getBaseJoystick() {
		return base;
	}
}