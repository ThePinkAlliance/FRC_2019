package frc.pink_233.commands;

import frc.pink_233.robot;

import edu.wpi.first.wpilibj.command.Command;

public class tank_drive extends Command {

		public tank_drive() {
			// TODO Auto-generated constructor stub
			requires(robot.drive_train);
		}
	
		// Called repeatedly when this Command is scheduled to run
		@Override
		protected void execute() {
			robot.drive_train.drive(robot.oi.getBaseJoystick());
		}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
			return false; // Runs until interrupted
		}

		// Called once after isFinished returns true
		@Override
		protected void end() {
			robot.drive_train.drive(0, 0);
		}
	

}