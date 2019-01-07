package frc.pink_233;

public class pink_pd {

    private pink_pd() {}

    // Use a PD to determine a motor command, which has a range of -1 to 1 (-1=rev; 0=stop; 1=fwd)
    public static double getMotorCmd(double Kp, double Kd, double error, double currentVel) {
        double motorCmd = (Kp * error) - (Kd * currentVel);
        return motorCmd;
    }

    // Use a PD to determine a continuous servo command, which has a range of 0 to 1 (0=rev; .5=stop; 1=fwd)
    public static double getServoCmd(double Kp, double Kd, double error, double currentVel) {

        double servoCmd;

        servoCmd = (Kp * error) - (Kd * currentVel);
        servoCmd = range.clip(servoCmd, 1.0, -1.0);

        return servoCmd;
    }
}