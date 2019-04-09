package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

// A Command which controls the DriveTrain subsystem using a Tank Drive control system
public class JoystickDrive extends Command {

	// Declare variables for this Command
	private Joystick js = Robot.m_oi.getBaseJoystick();
	private double right = 0.0;
	private double left = 0.0;

	// Construct this Command
	public JoystickDrive() {

		// Requires DriveTrain
		requires(Robot.m_driveTrain);
	}

	// Called repeatedly while this Command is running
	@Override protected void execute() {

		// Get the left joystick while accounting for mechanical deviation
			left = js.getRawAxis(OI.leftStick);

		// Get the right joystick while accounting for mechanical deviation
			right = js.getRawAxis(OI.rightStick);


			if( js.getRawButton(OI.leftBumperButtonNumber)) {
				AimAtTarget cmd = new AimAtTarget();
				cmd.start();
			  }

		// Use tank drive to move the base using the joystick values defined above
		Robot.m_driveTrain.tankDriveByJoystick(left, right);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override protected boolean isFinished() {

		// Return false so this Command runs until interrupted
		return false;
	}

	// Called once after isFinished returns true
	@Override protected void end() {

		// Stop the DriveTrain motors
		Robot.m_driveTrain.drivetrainStop();
	}

	// Called when another command which requires one or more of the same subsystems is scheduled to run
	@Override protected void interrupted() {

		// Stop the DriveTrain motors
		Robot.m_driveTrain.drivetrainStop();
	}
}