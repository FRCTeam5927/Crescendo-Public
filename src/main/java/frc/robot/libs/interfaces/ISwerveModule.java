package frc.robot.libs.interfaces;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {
    void setDesiredState(SwerveModuleState desiredState);
    void setModuleAngle(double angle);
    SwerveModuleState getModuleState();
    SwerveModulePosition getModulePosition();

    void tuneDrive();
    void tuneAngle();
    double getDriveTemperature();
    double getAngleTemperature();
}
