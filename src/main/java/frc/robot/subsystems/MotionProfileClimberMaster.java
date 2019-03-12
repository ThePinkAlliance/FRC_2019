package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.commands.ClimberDefaultMaster;
import frc.robot.subsystems.utils.Constants;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.ClimberDirection;
import frc.robot.subsystems.utils.MotionProfileClimberDouble.PodPosition;

/**
 * Add your docs here.
 */
public class MotionProfileClimberMaster extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX _fr = null;
  TalonSRX _fl = null;
  TalonSRX _br = null;
  TalonSRX _bl = null;

  
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

  public MotionProfileClimberMaster(int fr, int fl, int br, int bl, 
                                    PodPosition face, PodPosition side) {
    
    _fr = new TalonSRX(fr);
    _fl = new TalonSRX(fl);
    _br = new TalonSRX(br);
    _bl = new TalonSRX(bl);
    _example = new MotionProfileClimberDouble(_fr, null);
    setupTalon(_fr, PodPosition.FRONT, PodPosition.RIGHT);
    setupTalon(_fl, PodPosition.FRONT, PodPosition.LEFT);
    setupTalon(_br, PodPosition.BACK, PodPosition.RIGHT);
    setupTalon(_bl, PodPosition.BACK, PodPosition.LEFT);
  }

  
  public void setupTalon(TalonSRX talon, PodPosition face, PodPosition side) {
    /* Factory Default all hardware to prevent unexpected behaviour */
    talon.configFactoryDefault();
    talon.clearMotionProfileTrajectories(); // online
    talon.changeMotionControlFramePeriod(5);
    talon.setNeutralMode(NeutralMode.Brake);

    /**
     * the commands will call set inverted based on direction
     */
    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      setupRightFront();
    }
    if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      setupRightBack();
    }
    if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      setupLeftFront();
    }
    if (side == PodPosition.LEFT && face == PodPosition.BACK) {
      setupLeftBack();
    }

    /* Configure Selected Sensor for Motion Profile */

    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, frc.robot.subsystems.utils.Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
    /*
     * Keep sensor and motor in phase, postive sensor values when MC LEDs are green
     */

    /**
     * Configure MotorController Neutral Deadband, disable Motor Controller when
     * requested Motor Output is too low to process
     */
    talon.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

    /* Configure PID Gains, to be used with Motion Profile */

    // if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
    //   talon.config_kF(Constants.kPIDLoopIdx, Constants.kGainsRightFront.kF, Constants.kTimeoutMs);
    //   talon.config_kP(Constants.kPIDLoopIdx, Constants.kGainsRightFront.kP, Constants.kTimeoutMs);
    //   talon.config_kI(Constants.kPIDLoopIdx, Constants.kGainsRightFront.kI, Constants.kTimeoutMs);
    //   talon.config_kD(Constants.kPIDLoopIdx, Constants.kGainsRightFront.kD, Constants.kTimeoutMs);
    // }
    // if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
    //   talon.config_kF(Constants.kPIDLoopIdx, Constants.kGainsRightBack.kF, Constants.kTimeoutMs);
    //   talon.config_kP(Constants.kPIDLoopIdx, Constants.kGainsRightBack.kP, Constants.kTimeoutMs);
    //   talon.config_kI(Constants.kPIDLoopIdx, Constants.kGainsRightBack.kI, Constants.kTimeoutMs);
    //   talon.config_kD(Constants.kPIDLoopIdx, Constants.kGainsRightBack.kD, Constants.kTimeoutMs);    
    // }
    // if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
    //   talon.config_kF(Constants.kPIDLoopIdx, Constants.kGainsLeftFront.kF, Constants.kTimeoutMs);
    //   talon.config_kP(Constants.kPIDLoopIdx, Constants.kGainsLeftFront.kP, Constants.kTimeoutMs);
    //   talon.config_kI(Constants.kPIDLoopIdx, Constants.kGainsLeftFront.kI, Constants.kTimeoutMs);
    //   talon.config_kD(Constants.kPIDLoopIdx, Constants.kGainsLeftFront.kD, Constants.kTimeoutMs); 
    // }
    // if (side == PodPosition.LEFT && face == PodPosition.BACK) {
    //   talon.config_kF(Constants.kPIDLoopIdx, Constants.kGainsLeftBack.kF, Constants.kTimeoutMs);
    //   talon.config_kP(Constants.kPIDLoopIdx, Constants.kGainsLeftBack.kP, Constants.kTimeoutMs);
    //   talon.config_kI(Constants.kPIDLoopIdx, Constants.kGainsLeftBack.kI, Constants.kTimeoutMs);
    //   talon.config_kD(Constants.kPIDLoopIdx, Constants.kGainsLeftBack.kD, Constants.kTimeoutMs); 
    // }

    // _talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    // _talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    // _talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    // _talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /* Our profile uses 10ms timing */
    talon.configMotionProfileTrajectoryPeriod(10, Constants.kTimeoutMs);

    /*
     * Status 10 provides the trajectory target for motion profile AND motion magic
     */
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    talon.setSelectedSensorPosition(0);
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new ClimberDefaultMaster());
    //setDefaltCommand(new JoystickLevelClimber());
  }

  public void resetEncoderPosition(PodPosition face, PodPosition side) {
    
    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      _fr.setSelectedSensorPosition(0);
    } 
    else if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      _br.setSelectedSensorPosition(0);
      }
    else  if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      _fl.setSelectedSensorPosition(0);
      }
    else if (side == PodPosition.LEFT && face == PodPosition.BACK) {
      _bl.setSelectedSensorPosition(0);
    }
  }

  public int getEncPosition(PodPosition face, PodPosition side) {
    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      return _fr.getSelectedSensorPosition();
    } 
    else if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      return _br.getSelectedSensorPosition();
      }
    else  if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      return _fl.getSelectedSensorPosition();
      }
    else if (side == PodPosition.LEFT && face == PodPosition.BACK) {
      return _bl.getSelectedSensorPosition();
    } else {
      System.out.println("RETURNED ZERO");
      return 0;
    }
   
  }

  public int getEncVelocity(PodPosition face, PodPosition side) {
    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      return _fr.getSelectedSensorVelocity();//.getSelectedSensorPosition();
    } 
    else if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      return _br.getSelectedSensorVelocity();//.getSelectedSensorPosition();
      }
    else  if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      return _fl.getSelectedSensorVelocity();//.getSelectedSensorPosition();
      }
    else if (side == PodPosition.LEFT && face == PodPosition.BACK) {
      return _bl.getSelectedSensorVelocity();//.getSelectedSensorPosition();
    } else {
      System.out.println("RETURNED ZERO");
      return 0;
    }
   
  }

  public void set(PodPosition face, PodPosition side, double output) {
    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      _fr.set(ControlMode.PercentOutput, output);
    } 
    else if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      _br.set(ControlMode.PercentOutput, output);
      }
    else  if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      _fl.set(ControlMode.PercentOutput, output);
      }
    else if (side == PodPosition.LEFT && face == PodPosition.BACK) {
     _bl.set(ControlMode.PercentOutput, output);
    }
  }

  public void setPosition(PodPosition face, PodPosition side, double position) {
    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      _fr.set(ControlMode.Position, position);
    } 
    else if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      _br.set(ControlMode.Position, position);
      }
    else  if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      _fl.set(ControlMode.Position, position);
      }
    else if (side == PodPosition.LEFT && face == PodPosition.BACK) {
     _bl.set(ControlMode.Position, position);
    }
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

  public void setupLeftFront() {
    System.out.println("Setup Left Front Running");
    
      _fl.setInverted(true);
      _fl.setSensorPhase(true);
  }

  public void setupLeftBack() {
    
      _bl.setInverted(false);
  }

  public void setupRightFront() {
      _fr.setInverted(true);
      _fr.setSensorPhase(false);
  }

  public void setupRightBack() {
      _br.setInverted(true);
  }

  public void setDirection(ClimberDirection direction) {
    if (direction == ClimberDirection.UP) {
      // UP
          _fl.setInverted(true); 
          _fr.setInverted(true);
          _bl.setInverted(false); // need testing     
          _br.setInverted(true); // need testing

    } else {
      // DOWN
          _fl.setInverted(false); // test with false while phase reversed
          _fr.setInverted(false);
          _bl.setInverted(true); // need testing
          _br.setInverted(false); // need testing
    }
  }

  public double getOutput(PodPosition face, PodPosition side) {
    if (side == PodPosition.RIGHT && face == PodPosition.FRONT) {
      return _fr.getMotorOutputPercent();
    } 
    else if (side == PodPosition.RIGHT && face == PodPosition.BACK) {
      return _br.getMotorOutputPercent();
      }
    else  if (side == PodPosition.LEFT && face == PodPosition.FRONT) {
      return _fl.getMotorOutputPercent();
      }
    else //if (side == PodPosition.LEFT && face == PodPosition.BACK) {
      return _bl.getMotorOutputPercent();
    //}
  }
}
