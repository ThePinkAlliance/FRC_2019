package frc.pink_233;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.pink_233.subsystems.drive_train;
import frc.pink_233.oi;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class robot extends TimedRobot {
	
	
	// Define subsystem variables
	public static drive_train drive_train;
	public static oi oi;
	public static double delayTime;
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		drive_train = new drive_train();
		oi = new oi();
	}
	

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		// Reset any resources that are not needed
		robot.drive_train.disableDriveTrain();
	}

	@Override
	public void disabledPeriodic() {
		delayTime = SmartDashboard.getNumber("Autonomous delay", delayTime);
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		drive_train.getGyroRotation();
		System.out.println("Teleop Init");
		drive_train.resetEncoders();
		drive_train.setDriveTrainSafety(true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		log();
	}
	
	public void log() {
		SmartDashboard.putNumber("Left Encoder = ", drive_train.getLeftDistance());
		SmartDashboard.putNumber("Right Encoder = ", drive_train.getRightDistance());
		SmartDashboard.putNumber("Left Raw = ", drive_train.leftEncoder.getRaw());
		SmartDashboard.putNumber("Right Raw = ", drive_train.rightEncoder.getRaw());
		SmartDashboard.putData("Gyro", drive_train.getDriveTrainGyro());
		SmartDashboard.putNumber("Gyro rate ", drive_train.getGyroRate());
		SmartDashboard.putNumber("Gyro angle ", drive_train.getGyroRotation());
	}
	
	@Override
	public void robotPeriodic() {
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}