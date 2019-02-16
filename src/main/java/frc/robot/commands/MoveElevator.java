package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

// Command used to move the Elevator
public class MoveElevator extends Command {
  // Declare Command Variables
  private Joystick js = null;
  private double stickValue = 0;

  

  // Command Constructor
  public MoveElevator() {
    // Declare Subsystem Dependencies
    requires(Robot.m_elevator);
    // Define Command Varibales
    js = Robot.m_oi.getTowerJoystick();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    ReadJoystick();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevator._elevator.set(0);
  }

  // Called when another command which requires one or more of the same subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_elevator._elevator.set(0);
  }

  // Method to set motor power based of the stickValue
  public void ReadJoystick() {
    // Read out stickValue
    stickValue = js.getRawAxis(OI.rightStick);

    if(stickValue > -0.1 && stickValue < 0.1) {
      stickValue = 0;
    }
      
    // Set _elevator Motor to stickValue
    Robot.m_elevator.moveElevator(stickValue);
    System.out.println("Elevator Power: " + stickValue);
  }
}