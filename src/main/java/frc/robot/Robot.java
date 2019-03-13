package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberDriver;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hatch;
import frc.robot.subsystems.MotionProfileClimberMaster;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;
import frc.robot.subsystems.Ball;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Ball m_ball = new Ball();
  public static DriveTrain m_driveTrain = new DriveTrain();
  public static ClimberDriver m_climberDriver = new ClimberDriver();
  public static Climber m_climber = new Climber();
  
  /*public static MotionProfileClimber m_climberPodFrontLeft = new MotionProfileClimber(RobotMap.leftFrontClimberPort,
                                                                                      MotionProfileClimber.TALON_ID_NULL,
                                                                                      //RobotMap.rightFrontClimberPort,
                                                                                      MotionProfileClimber.PodPosition.FRONT, 
                                                                                      MotionProfileClimber.PodPosition.LEFT);
  
  public static MotionProfileClimber m_climberPodBackLeft = new MotionProfileClimber(RobotMap.leftBackClimberPort,
                                                                                      MotionProfileClimber.TALON_ID_NULL,
                                                                                      //RobotMap.rightBackClimberPort,
                                                                                      MotionProfileClimber.PodPosition.BACK, 
                                                                                      MotionProfileClimber.PodPosition.LEFT); 
                                                                                      
  public static MotionProfileClimber m_climberPodBackRight = new MotionProfileClimber(RobotMap.rightBackClimberPort,
                                                                                      MotionProfileClimber.TALON_ID_NULL,
                                                                                      MotionProfileClimber.PodPosition.BACK, 
                                                                                      MotionProfileClimber.PodPosition.RIGHT);

  public static MotionProfileClimber m_climberPodFrontRight = new MotionProfileClimber(RobotMap.rightFrontClimberPort,
                                                                                      MotionProfileClimber.TALON_ID_NULL,
                                                                                      MotionProfileClimber.PodPosition.FRONT, 
                                                                                      MotionProfileClimber.PodPosition.RIGHT);
   */                                                                                     
     
  
  public static MotionProfileClimberMaster m_climberMaster = new MotionProfileClimberMaster(RobotMap.rightFrontClimberPort,
                                                                                            RobotMap.leftFrontClimberPort, 
                                                                                            RobotMap.rightBackClimberPort, 
                                                                                            RobotMap.leftBackClimberPort, 
                                                                                            PodPosition.RIGHT, PodPosition.FRONT);
  public static Hatch m_hatch = new Hatch();
  public static Elevator m_elevator = new Elevator();
  public static OI m_oi;
  public static RobotDashboard m_rDashboard;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    //Joystick
    m_oi = new OI();

    //Dashboard
    m_rDashboard = new RobotDashboard();
    m_rDashboard.displayEncoderValues();
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    //DriveTrain


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    m_rDashboard.displayContinuousData();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    //Robot.m_driveTrain.drive_to_distance(20);

    m_rDashboard.displayContinuousData();
    m_rDashboard.getContinuousData();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    m_rDashboard.displayContinuousData();
    m_rDashboard.getContinuousData();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
