package frc.pink_233.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.pink_233.Robot_Configuration;

public class drive_train {
    static SpeedController left_front_motor = new CANSparkMax(Robot_Configuration.left_front_motor_port, MotorType.kBrushless);
    static SpeedController left_back_motor = new CANSparkMax(Robot_Configuration.left_back_motor_port, MotorType.kBrushless);
    static SpeedController right_front_motor = new CANSparkMax(Robot_Configuration.right_front_motor_port, MotorType.kBrushless);
    static SpeedController right_back_motor = new CANSparkMax(Robot_Configuration.right_back_motor_port, MotorType.kBrushless);
    
    public static SpeedControllerGroup left_motors = new SpeedControllerGroup(left_front_motor, left_back_motor);
    public static SpeedControllerGroup right_motors = new SpeedControllerGroup(right_front_motor, right_back_motor);
}