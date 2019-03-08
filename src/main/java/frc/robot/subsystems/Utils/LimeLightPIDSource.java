package frc.robot.subsystems.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class LimeLightPIDSource implements PIDSource {

    private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;


@Override
public PIDSourceType getPIDSourceType() {
    return pidSourceType;
}


// @Override
public double pidGet() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
}

@Override
  public void setPIDSourceType(PIDSourceType pidSourceType) {
    this.pidSourceType = pidSourceType;
  }

}