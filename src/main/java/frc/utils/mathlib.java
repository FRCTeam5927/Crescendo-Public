package frc.utils;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.Drivetrain.OIConstants;

public class mathlib {
    public static double applyDriveDeadband(double i) {
        return -MathUtil.applyDeadband(i, OIConstants.kDriveDeadband);
    }
}
