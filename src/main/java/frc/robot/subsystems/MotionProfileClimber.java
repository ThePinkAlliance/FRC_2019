package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.commands.ClimberDefault;
import frc.robot.subsystems.utils.Constants;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;

/**
 * Add your docs here.
 */
public class MotionProfileClimber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX _talon1 = null;
  TalonSRX _talon2 = null;

  public final static boolean SWITCH_CLOSED = true;
  public final static boolean SWITCH_OPEN = false;

  /** some example logic on how one can manage an MP */
  MotionProfileClimberDouble _example = null;

  private PodPosition face = PodPosition.FRONT;
  private PodPosition side = PodPosition.LEFT;
  private double startingPosition = 0.0;

  // // point of view of the face that we climb with
  // public static enum PodPosition {
  //   FRONT, // collector side
  //   BACK, // beak side
  //   LEFT, // where front is the collector
  //   RIGHT // where front is the collector
  // }

  // // point of view of the robot, not the pod mechanisms
  // // e.g.: when the robot belly pan rises, thats up
  // public static enum ClimberDirection {
  //   UP, DOWN
  // }

  // NOTALON ID for instance without a follower
  public final static int TALON_ID_NULL = -1;

  public MotionProfileClimber(int talon1, int talon2, PodPosition face, PodPosition side) {
    this.face = face;
    this.side = side;
    _talon1 = new TalonSRX(talon1);
    if (talon2 != TALON_ID_NULL)
      _talon2 = new TalonSRX(talon2);
    _example = new MotionProfileClimberDouble(_talon1, _talon2);
    setupTalon();
  }

  public void invertTalon1(boolean invert) {
    _talon1.setInverted(invert);
  }

  public void invertTalon2(boolean invert) {
    _talon2.setInverted(invert);
  }

  public void setupTalon() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    _talon1.configFactoryDefault();
    if (_talon2 != null) {
      _talon2.configFactoryDefault();
      _talon2.set(ControlMode.Follower, _talon1.getDeviceID());
    }
    _talon1.clearMotionProfileTrajectories(); // online

    _talon1.changeMotionControlFramePeriod(5);
    if (_talon2 != null) {
      _talon2.changeMotionControlFramePeriod(5);
      _talon1.setNeutralMode(NeutralMode.Brake);

    }

    _talon1.setNeutralMode(NeutralMode.Brake);

    /**
     * the commands will call set inverted based on direction
     */
    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      setupRightFront(null);
      if (_talon2 != null) {
        setupLeftFront(_talon2);
      }
    }
    if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      setupRightBack(null);
      if (_talon2 != null) {
        setupLeftBack(_talon2);
      }
    }
    if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      setupLeftFront(null);
      if (_talon2 != null) {
        setupRightFront(_talon2);
      }
    }
    if (side == PodPosition.LEFT && face == PodPosition.BACK) {
      setupLeftBack(null);
      if (_talon2 != null) {
        setupRightBack(_talon2);
      }
    }

    /* Configure Selected Sensor for Motion Profile */

    _talon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, frc.robot.subsystems.utils.Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
    /*
     * Keep sensor and motor in phase, postive sensor values when MC LEDs are green
     */

    /**
     * Configure MotorController Neutral Deadband, disable Motor Controller when
     * requested Motor Output is too low to process
     */
    _talon1.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

    /* Configure PID Gains, to be used with Motion Profile */

    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      _talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGainsRightFront.kF, Constants.kTimeoutMs);
      _talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGainsRightFront.kP, Constants.kTimeoutMs);
      _talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGainsRightFront.kI, Constants.kTimeoutMs);
      _talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGainsRightFront.kD, Constants.kTimeoutMs);
    }
    if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      _talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGainsRightBack.kF, Constants.kTimeoutMs);
      _talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGainsRightBack.kP, Constants.kTimeoutMs);
      _talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGainsRightBack.kI, Constants.kTimeoutMs);
      _talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGainsRightBack.kD, Constants.kTimeoutMs);    
     
    }
    if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      _talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGainsLeftFront.kF, Constants.kTimeoutMs);
      _talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGainsLeftFront.kP, Constants.kTimeoutMs);
      _talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGainsLeftFront.kI, Constants.kTimeoutMs);
      _talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGainsLeftFront.kD, Constants.kTimeoutMs); 
    }
    if (side == PodPosition.LEFT && face == PodPosition.BACK) {
      _talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGainsLeftBack.kF, Constants.kTimeoutMs);
      _talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGainsLeftBack.kP, Constants.kTimeoutMs);
      _talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGainsLeftBack.kI, Constants.kTimeoutMs);
      _talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGainsLeftBack.kD, Constants.kTimeoutMs); 
    }

    // _talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    // _talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    // _talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    // _talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /* Our profile uses 10ms timing */
    _talon1.configMotionProfileTrajectoryPeriod(10, Constants.kTimeoutMs);

    /*
     * Status 10 provides the trajectory target for motion profile AND motion magic
     */
    _talon1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    resetEncoderPosition(0);
    startingPosition = 0;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ClimberDefault(this));
  }

  public void resetEncoderPosition(int position) {
    _talon1.setSelectedSensorPosition(0);
  }

  public double getEncPosition() {
    return _talon1.getSelectedSensorPosition();
  }

  public void set(double output) {
    _talon1.set(ControlMode.PercentOutput, output);
  }

  public boolean isMotionProfileFinished() {
    return _example.isMotionProfileDone();
  }

  public MotionProfileClimberDouble getMP() {
    return _example;
  }

  public PodPosition getFace() {
    return face;
  }

  public PodPosition getSide() {
    return side;
  }

  public void setupLeftFront(TalonSRX follower) {
    System.out.println("Setup Left Front Running");
    if (follower == null) {
      _talon1.setInverted(true);
      _talon1.setSensorPhase(true);
    } else {
      follower.setInverted(true);
      follower.setSensorPhase(true);
    }
  }

  public void setupLeftBack(TalonSRX follower) {
    if (follower == null) {
      _talon1.setInverted(false);
      // on practice bot default phase was right
    } else {
      follower.setInverted(false);
      // on practice bot default phase was right
    }
  }

  public void setupRightFront(TalonSRX follower) {
    if (follower == null) {
      _talon1.setInverted(true);
      _talon1.setSensorPhase(false);
    } else {
      follower.setInverted(true);
      follower.setSensorPhase(false);
    }
  }

  public void setupRightBack(TalonSRX follower) {
    if (follower == null) {
      _talon1.setInverted(true);
      // on practice bot default phase was right
    } else {
      follower.setInverted(true);
      // on practice bot default phase was right
    }
  }

  public void setDirection(ClimberDirection direction) {
    if (direction == ClimberDirection.UP) {
      // UP
      if (face == PodPosition.FRONT) {
        // FRONT
        if (side == PodPosition.LEFT) {
          // LEFT
          _talon1.setInverted(true); // test with false while phase reversed
          if (_talon2 != null)
            _talon2.setInverted(true);
        } else {
          // RIGHT
          _talon1.setInverted(true);
          if (_talon2 != null)
            _talon2.setInverted(true);
        }

      } else {
        // BACK
        if (side == PodPosition.LEFT) {
          // LEFT
          _talon1.setInverted(false); // need testing
          if (_talon2 != null)
            _talon2.setInverted(true);
        } else {
          // RIGHT
          _talon1.setInverted(true); // need testing
          if (_talon2 != null)
            _talon2.setInverted(false);
        }
      }

    } else {
      // DOWN
      if (face == PodPosition.FRONT) {
        // FRONT
        if (side == PodPosition.LEFT) {
          // LEFT
          _talon1.setInverted(false); // test with false while phase reversed
          if (_talon2 != null)
            _talon2.setInverted(false);
        } else {
          // RIGHT
          _talon1.setInverted(false);
          if (_talon2 != null)
            _talon2.setInverted(false);
        }

      } else {
        // BACK
        if (side == PodPosition.LEFT) {
          // LEFT
          _talon1.setInverted(true); // need testing
          if (_talon2 != null)
            _talon2.setInverted(false);
        } else {
          // RIGHT
          _talon1.setInverted(false); // need testing
          if (_talon2 != null)
            _talon2.setInverted(true);
        }
      }
    }
  }

  public double getOutput() {
    return _talon1.getMotorOutputPercent();
  }
  // public void setDirection(ClimberDirection direction) {
  // if (side == PodPosition.LEFT) {
  // if (direction == ClimberDirection.UP) {
  // if (face == PodPosition.FRONT) {
  // _talon1.setInverted(true);
  // } else {
  // _talon1.setInverted(false);
  // }
  // } else {
  // if (direction == ClimberDirection.UP) {
  // _talon1.setInverted(false);
  // } else {
  // _talon1.setInverted(true);
  // }
  // }
  // }
}
