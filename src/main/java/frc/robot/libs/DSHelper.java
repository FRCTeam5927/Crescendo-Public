package frc.robot.libs;

import edu.wpi.first.wpilibj.DriverStation;

public class DSHelper {

    public static DriverStation.Alliance getAlliance() {
        if(DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get();
        } else {
            return DriverStation.Alliance.Blue;
        }
    }
}
