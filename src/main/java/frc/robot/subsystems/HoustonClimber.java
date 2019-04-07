package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.commands.HoustonClimberTestJoystick;
import frc.robot.subsystems.utils.Constants;
import frc.robot.subsystems.utils.HoustonMotionProfileExecutor;
import edu.wpi.first.wpilibj.command.Subsystem;
//import frc.robot.subsystems.utils.MotionProfileClimberDouble;
//import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
//import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;

/**
 * Add your docs here.
 */
public class HoustonClimber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // point of view of the face that we climb with
	public static enum PodPosition {
		LEFT, // where front is the collector
		RIGHT // where front is the collector
	}

	// point of view of the robot, not the pod mechanisms
	// e.g.: when the robot belly pan rises, thats up
	public static enum PodAction {
		CLIMB, RETRACT
	}

	// which platform?
	// e.g.: when the robot belly pan rises, thats up
	public static enum ClimbLevel {
		LEVEL2, LEVEL3
	}

  TalonSRX _master = null;
  TalonSRX _follower = null;
  PodPosition _masterPosition = PodPosition.LEFT;
  PodPosition _followerPosition = PodPosition.RIGHT;
  

  
  /** some example logic on how one can manage an MP */
  HoustonMotionProfileExecutor _example = null;

  
  // NO TALON ID for instance without a follower
  public final static int TALON_ID_NULL = -1;

  public HoustonClimber(int left, int right, PodPosition masterPosition) {
    
    if (masterPosition == PodPosition.LEFT) {
       _master = new TalonSRX(left);
       _follower = new TalonSRX(right);
       _masterPosition = PodPosition.LEFT;
       _followerPosition = PodPosition.RIGHT;
    } else {
      _master = new TalonSRX(right);
      _follower = new TalonSRX(left);
      _masterPosition = PodPosition.RIGHT;
      _followerPosition = PodPosition.LEFT;
    }
    
    _example = new HoustonMotionProfileExecutor(_master, _follower);
    

    setupTalons();
  }

  
  public void setupTalons() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    // Talon1 is expected to be the ESC that we use to read encoder data from
    _master.configFactoryDefault();
    _master.setNeutralMode(NeutralMode.Brake);
    // Talon2 is expected to be the follower
    _follower.configFactoryDefault();
    _follower.setNeutralMode(NeutralMode.Brake);
    _follower.set(ControlMode.Follower, _master.getDeviceID());

    _master.clearMotionProfileTrajectories(); // online
    _master.changeMotionControlFramePeriod(5);
    _follower.changeMotionControlFramePeriod(5);

    /* Configure Selected Sensor for Motion Profile */
    _master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    _follower.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
    /*
     * Keep sensor and motor in phase, postive sensor values when MC LEDs are green
     */

    /**
     * Configure MotorController Neutral Deadband, disable Motor Controller when
     * requested Motor Output is too low to process
     */
    _master.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

    /* Configure PID Gains, to be used with Motion Profile */
    _master.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    _master.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    _master.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _master.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    

    /* Our profile uses 10ms timing */
    _master.configMotionProfileTrajectoryPeriod(10, Constants.kTimeoutMs);

    /*
     * Status 10 provides the trajectory target for motion profile AND motion magic
     */
    _master.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    //Setup initial invert and phase (default to climb invert, 
    //phase only set once - green led must generate positive encoder tick counts)
    setupLeftDefault();
    setupRightDefault();

    resetEncoderPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new up your command here);
    //setDefaultCommand(new HoustonClimberTestJoystick());
  }

  public void resetEncoderPosition(int position) {
    _master.setSelectedSensorPosition(position);
    _follower.setSelectedSensorPosition(position);
  }

  public double getEncPositionMaster() {
    return _master.getSelectedSensorPosition();
  }

  public double getEncPositionFollower() {
    return _follower.getSelectedSensorPosition();
  }

  public void set(double output) {
    _master.set(ControlMode.PercentOutput, output);
  }

  public void setPosition(double position) {
    _master.set(ControlMode.Position, position);
  }

  public boolean isMotionProfileFinished() {
    return _example.isMotionProfileDone();
  }

  public HoustonMotionProfileExecutor getMP() {
    return _example;
  }

  public void setupLeftDefault() {
    if (_masterPosition == PodPosition.LEFT) {
      _master.setInverted(false);
      _master.setSensorPhase(true); 
    } else {
      _follower.setInverted(false);
      _follower.setSensorPhase(true);
    }
  }

  public void setupRightDefault() {
    if (_masterPosition == PodPosition.RIGHT) {
      _master.setInverted(false);
      _master.setSensorPhase(false);  //TODO: this is different on competition bot
    } else {
      _follower.setInverted(false);
      _follower.setSensorPhase(false);  //TODO: this is different on competition bot
    }
  }

  public void setupClimb() {
    //LEFT
    if (_masterPosition == PodPosition.LEFT) {
      _master.setInverted(false);
    } else {
      _follower.setInverted(false);
    }
    //RIGHT
    if (_masterPosition == PodPosition.RIGHT) {
      _master.setInverted(false);
    } else {
      _follower.setInverted(false);
    }
  }

  public void setupRetract() {
    
    //LEFT
    if (_masterPosition == PodPosition.LEFT) {
      _master.setInverted(true);
    } else {
      _follower.setInverted(true);
    }
    //RIGHT
    if (_masterPosition == PodPosition.RIGHT) {
      _master.setInverted(true);
    } else {
      _follower.setInverted(true);
    }
  }

  public void setAction(PodAction action) {
    if (action == PodAction.CLIMB) {
      setupClimb();
    } else {
      setupRetract();
    }   
  }

  public double getOutput() {
    return _master.getMotorOutputPercent();
  }
}
