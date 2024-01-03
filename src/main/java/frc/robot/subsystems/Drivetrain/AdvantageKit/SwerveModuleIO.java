package frc.robot.subsystems.Drivetrain.AdvantageKit;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

    @AutoLog
    public class SwerveModuleIOInputs {

        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTemperatureCelsius = 0.0;

        public double turnAbsolutePosition = 0.0;
        public double turnPosition = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTemperatureCelsius = 0.0;

    }

    default void updateInputs(SwerveModuleIOInputs inputs) {}

    SwerveModuleIOInputs getInputs();

    void setSpeed(double speed);
    void setAngle(double angle);
    SwerveModuleState optimize(SwerveModuleState swerveModuleState);
    void stopDrive();
    void stopAngle();
    void tuneDrive();
    void tuneAngle();

}
