package frc.robot.subsystems.Drivetrain.AdvantageKit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOManager;
import frc.robot.libs.interfaces.ISwerveModule;

public class SwerveModule extends SubsystemBase implements ISwerveModule {
    SwerveModuleIO moduleIO;
    int modID;

    public SwerveModule(int id) {
        moduleIO = IOManager.getSwerveModule(id);
        modID = id;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("id"+modID, moduleIO.getInputs().turnAbsolutePosition);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {


        SwerveModuleState optimizedDesiredState = moduleIO.optimize(desiredState);


        double optimizedMPS = optimizedDesiredState.speedMetersPerSecond;
        double optimizedRadians = optimizedDesiredState.angle.getRadians();

        SmartDashboard.putString("opstate"+modID, optimizedDesiredState.toString());


        moduleIO.setSpeed(optimizedMPS);
        if(Math.abs(optimizedMPS) < 0.01) {
            moduleIO.setAngle(optimizedRadians);
        }
    }

    @Override
    public void setModuleAngle(double angle) {
        moduleIO.setAngle(angle);
        moduleIO.stopDrive();
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                moduleIO.getInputs().driveVelocityMetersPerSec,
                new Rotation2d(moduleIO.getInputs().turnAbsolutePosition)
        );
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                moduleIO.getInputs().drivePositionMeters,
                new Rotation2d(moduleIO.getInputs().turnAbsolutePosition)
        );
    }

    @Override
    public void tuneDrive() {
        moduleIO.tuneDrive();
    }

    @Override
    public void tuneAngle() {
        moduleIO.tuneAngle();
    }

    @Override
    public double getDriveTemperature() {
        return moduleIO.getInputs().driveTemperatureCelsius;
    }

    @Override
    public double getAngleTemperature() {
        return moduleIO.getInputs().turnTemperatureCelsius;
    }
}
