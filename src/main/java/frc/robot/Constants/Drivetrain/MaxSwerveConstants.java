package frc.robot.Constants.Drivetrain;

import edu.wpi.first.math.util.Units;

public class MaxSwerveConstants {
    public static final double SWERVE_GEAR_RATIO = 6.86;
    public static final double WHEEL_DIAMETER = (3.76);
    public static final double WHEEL_CIRCUMFERENCE = ((WHEEL_DIAMETER *Math.PI)/12);
    public static final double TUNING_CONSTANT = 1.027131474;
    public static final double WHEEL_EFFECTIVE_CIRCUMFERENCE = Units.feetToMeters(WHEEL_CIRCUMFERENCE/SWERVE_GEAR_RATIO)*TUNING_CONSTANT;
}
