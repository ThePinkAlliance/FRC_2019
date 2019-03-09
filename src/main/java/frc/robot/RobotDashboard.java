package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// The RobotDashboard is a helper class to encapsulate 
// displaying data to the dashboard.
public class RobotDashboard {
  // Encoder Data
  public static final String ENC_LABEL_RIGHT_FRONT = "Right Front Distance(in)";
  public static final String ENC_LABEL_LEFT_FRONT = "Left Front Distance(in)";
  public static final String ENC_LABEL_FRONT = "Front Distance Average(in)";
  public static final String ENC_LABEL_ELEVATOR = "Elevator Height (Encoder Counts)";

  // PID for EncoderBasedDrive
  public static final String DT_ENC_PID_DISTANCE = "DT_ENC_PID_DISTANCE";
  public static final String DT_ENC_PID_MAX_OUTPUT = "DT_ENC_PID_OUTPUT";
  public static final String DT_ENC_Kp = "DT_ENC_Kp";
  public static final String DT_ENC_Ki = "DT_ENC_Ki";
  public static final String DT_ENC_Kd = "DT_ENC_Kd";

  // Gyro Data
  public static final String NAVX_LABEL_ANGLE = "NAVX Angle";
  public static final String NAVX_LABEL_YAW = "NAVX Yaw";

  // PID for DriveStraightByGyro
  public static final String DT_NAVX_PID_ANGLE = "DT_NAVX_PID_ANGLE";
  public static final String DT_NAVX_PID_MAX_OUTPUT = "DT_NAVX_PID_OUTPUT";
  public static final String DT_NAVX_Kp = "DT_NAVX_Kp";
  public static final String DT_NAVX_Ki = "DT_NAVX_Ki";
  public static final String DT_NAVX_Kd = "DT_NAVX_Kd";

  // PD for DriveTrain
  public static final String BASE_KP = "Base Kp Value";
  public static final String BASE_KD = "Base Kd Value";

  // Should the Dashboard Display Data?
  private boolean bDisplayContinuousData = true;

  // Constructor that puts values onto dashboard for the first time.
  public RobotDashboard() {
    // Handle initial values to seed the dashboard.
    displayInitialValues();
  }

  /**
  * Can be called in Robot.java or anywhere else to turn off or on the calls
  * to put data on the dashboard.
  * @param bDisplay
  */
  public void setDisplayContinuousData(boolean bDisplay) {
    this.bDisplayContinuousData = bDisplay;
  }

  /**
  * Can be checked to see if we are putting data on the dashboard
  * @return bDisplayContinuousData
  */
  public boolean getDisplayContinuousData() {
    return this.bDisplayContinuousData;
  }

    /**
     * Prints any encoder related values to the dashboard
     */
    public void displayEncoderValues() {
        if (Robot.m_driveTrain != null) {
            SmartDashboard.putNumber(ENC_LABEL_RIGHT_FRONT, Robot.m_driveTrain.getFrontRightPosition());
            SmartDashboard.putNumber(ENC_LABEL_LEFT_FRONT, Robot.m_driveTrain.getFrontLeftPosition());
            SmartDashboard.putNumber(ENC_LABEL_ELEVATOR, Robot.m_elevator.getElevatorHeight());
            SmartDashboard.putNumber("PodFrontLeftEncoder", Robot.m_climberPodFrontLeft.getEncPosition());
            SmartDashboard.putNumber("PodBackLeftEncoder", Robot.m_climberPodBackLeft.getEncPosition());
           // SmartDashboard.putNumber("PodFrontRightEncoder", Robot.m_climberPodFrontRight.getEncPosition());
           // SmartDashboard.putNumber("PodBackRightEncoder", Robot.m_climberPodBackRight.getEncPosition());
        }
    }

    /**
     * Prints any gyro related values to the dashboard
     */
    public void displayGyroValues() {
        if (Robot.m_driveTrain != null) {
            SmartDashboard.putNumber(NAVX_LABEL_ANGLE, Robot.m_driveTrain.getGyroAngle());
            SmartDashboard.putNumber(NAVX_LABEL_YAW, Robot.m_driveTrain.getGyroYaw());
        }
    }

    public void displayPIDValues() {
        if (Robot.m_driveTrain != null) {
            SmartDashboard.putNumber(BASE_KD, Robot.m_driveTrain.baseKd);
            SmartDashboard.putNumber(BASE_KP, Robot.m_driveTrain.baseKp);
        }
    }

    public void getPIDValues() {
        if (Robot.m_driveTrain != null) {
            Robot.m_driveTrain.baseKd = SmartDashboard.getNumber(BASE_KD, Robot.m_driveTrain.baseKd);
            Robot.m_driveTrain.baseKp = SmartDashboard.getNumber(BASE_KP, Robot.m_driveTrain.baseKp);
        }
    }

    public void displaySwitchValues() {
        // SmartDashboard.putBoolean("Elevator Bottom Limit", Robot.m_elevator.getElevatorBottomSwitch());
        // SmartDashboard.putBoolean("Elevator Top Limit", Robot.m_elevator.getElevatorTopSwitch());
    }

    /**
     * Grabs values from the dashboard on a continuous basis (telelop / autonomous)
     */
    public void getContinuousData() {
        getPIDValues();
    }

    /**
     * Prints one time values to the dashboard to establish the widget
     */
    public void displayInitialValues() {
        displayEncoderValues();
        displayGyroValues();
        displayPIDValues();
        displaySwitchValues();
    }

    /**
     * Called from Robot.java during periodic states (teleop / autonomous)
     */
    public void displayContinuousData() {
        displayEncoderValues();
        displayGyroValues();
        displayPIDValues();
        displaySwitchValues();
    }
}