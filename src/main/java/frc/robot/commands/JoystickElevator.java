package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

// A Command which controls the DriveTrain subsystem using a Tank Drive control system
public class JoystickElevator extends Command {

	// Declare variables for this Command
	private Joystick js = Robot.m_oi.getTowerJoystick();

	// Construct this Command
	public JoystickElevator() {

		// Requires DriveTrain
		requires(Robot.m_elevator);
	}

	// Called repeatedly while this Command is running
	@Override protected void execute() {
    if(js.getRawAxis(OI.rightStick) > .1 || js.getRawAxis(OI.rightStick) < -0.1)
    {
    Robot.m_elevator._elevator.set(js.getRawAxis(OI.rightStick));
    } else{
      Robot.m_elevator._elevator.set(0.166);
    }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override protected boolean isFinished() {

		// Return false so this Command runs until interrupted
		return false;
	}

	// Called once after isFinished returns true
	@Override protected void end() {
	}

	// Called when another command which requires one or more of the same subsystems is scheduled to run
	@Override protected void interrupted() {
	}
}