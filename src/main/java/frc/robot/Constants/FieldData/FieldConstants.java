package frc.robot.Constants.FieldData;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.libs.DSHelper;
import frc.robot.libs.MathLib;

/**
 *
 */
public class FieldConstants {


    public static double getAllianceOffset() {
        DriverStation.Alliance alliance = DSHelper.getAlliance();
        if(alliance == DriverStation.Alliance.Blue) {
            return 0.5*Math.PI;
        } else if (alliance == DriverStation.Alliance.Red) {
            return -0.5*Math.PI;
        } else {
            return 0;
        }
    }


}
