/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.pink_233;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.pink_233.commands.encoder_based_drive;

/**
 * RobotDashboard: helper class to encapsulate displaying data to the dashboard.
 * This class has a boolean that can be set to false to stop displaying data.
 * 
 * The class instance is created after all subsystems and m_oi is created in Robot.java.
 * This ensures that we can access the subsystems to get their data and put it onto the
 * SmartDashboard.
 */
public class robot_dashboard {
    //Encoder data
    public static final String ENC_LABEL_RIGHT_FRONT =  "Right Front Distance(in)  ";
    public static final String ENC_LABEL_LEFT_FRONT  =  "Left Front Distance(in)   ";
    public static final String ENC_LABEL_FRONT       =  "Front Distance Average(in)";

    //PID for EncoderBasedDrive
    public static final String DT_ENC_PID_DISTANCE     = "DT_ENC_PID_DISTANCE";
    public static final String DT_ENC_PID_MAX_OUTPUT   = "DT_ENC_PID_OUTPUT";
    public static final String DT_ENC_Kp = "DT_ENC_Kp";
    public static final String DT_ENC_Ki = "DT_ENC_Ki";
    public static final String DT_ENC_Kd = "DT_ENC_Kd";

    
    private boolean bDisplayContinuousData = true;

    /**
     * Constructor:  put values onto dashboard for the first time.
     */
    public robot_dashboard() {
        //Handle initial values to seed the dashboard
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
        if (robot.drive_train != null) {
            SmartDashboard.putNumber(ENC_LABEL_RIGHT_FRONT, robot.drive_train.getFrontRightDistance());
            SmartDashboard.putNumber(ENC_LABEL_LEFT_FRONT, robot.drive_train.getFrontLeftDistance());
            SmartDashboard.putNumber(ENC_LABEL_FRONT, robot.drive_train.getFrontDistanceAverage());
        }
    }

    /**
     * Display DriveTrain Encoder PID values
     */
    public void displayDriveTrainEncoderPIDValues() {
        //DriveTrain subsystem
        if (robot.drive_train != null) {
            SmartDashboard.putNumber(DT_ENC_PID_DISTANCE, encoder_based_drive.CMD_DEFAULT_DISTANCE);
            SmartDashboard.putNumber(DT_ENC_PID_MAX_OUTPUT, encoder_based_drive.CMD_MAX_OUTPUT);
            SmartDashboard.putNumber(DT_ENC_Kp, encoder_based_drive.CMD_Kp);
            SmartDashboard.putNumber(DT_ENC_Ki, encoder_based_drive.CMD_Ki);
            SmartDashboard.putNumber(DT_ENC_Kd, encoder_based_drive.CMD_Kd);
        }
    }

    /**
     * Grab the DriveTrain Encoder PID values from the Dashboard and set them 
     * on the drive train
     */
    public void getDriveTrainEncoderPIDValues() {
        //DriveTrain subsystem
        if (robot.drive_train != null) {
            robot.drive_train.setEncKp(SmartDashboard.getNumber(DT_ENC_Kp, encoder_based_drive.CMD_Kp));
            robot.drive_train.setEncKi(SmartDashboard.getNumber(DT_ENC_Ki, encoder_based_drive.CMD_Ki));
            robot.drive_train.setEncKd(SmartDashboard.getNumber(DT_ENC_Kd, encoder_based_drive.CMD_Kd));
        }
    }

    /**
     * Grabs values from the dashboard on a continuous basis (telelop / autonomous)
     */
    public void getContinuousData() {
        getDriveTrainEncoderPIDValues();
    }

    /**
     * Prints one time values to the dashboard to establish the widget
     */
    public void displayInitialValues() {
        displayDriveTrainEncoderPIDValues();
        displayEncoderValues();
    }

    /**
     * Called from Robot.java during periodic states (teleop / autonomous)
     */
    public void displayContinuousData() {
        displayEncoderValues();
    }
}