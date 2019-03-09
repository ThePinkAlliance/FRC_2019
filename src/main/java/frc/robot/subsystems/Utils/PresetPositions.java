package frc.robot.subsystems.utils;

public class PresetPositions {

    // Ball Presets 0 is stowed and approximately 90000 is the lowest
    public static double BALL_COLLECT_POSITION = 60000;
    public static double BALL_LOW_ROCKET_POSITION = 30000;
    public static double BALL_CARGO_POSITION = 0;

    // Elevator Presets 0 is lowest and -50 is the highest
    public static double ELEVATOR_COLLECT_POSITION = 0;
    public static double ELEVATOR_MID_ROCKET_POSITION = -44;

    // Base Right Trigger is a toggle which will lower the ball collector to 
    //  collect position and collect until there is a ball at which point it 
    //  will raise to cargo automatically

    // Base Right Bumper is a manual toggle for collecting

    // Tower Up Dpad is cargo pos for ball collector

    // Tower Right Dpad is mid pos for ball collector

    // Tower Down Dpad is collect pos for ball collector

    // TowerX goes to mid hatch pos for elevator

    // TowerA goes to collect hatch pos for elevator
}
