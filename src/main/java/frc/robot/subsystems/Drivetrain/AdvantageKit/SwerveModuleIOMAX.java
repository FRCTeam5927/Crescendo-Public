package frc.robot.subsystems.Drivetrain.AdvantageKit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.Constants.Drivetrain.ModuleConstants;
import frc.robot.libs.BaseMechanism.GenericIdleMode;
import frc.robot.libs.BaseMechanism.SparkMAXMechanism;

import java.security.InvalidParameterException;

public class SwerveModuleIOMAX extends SubsystemBase implements SwerveModuleIO {
    SparkMAXMechanism driveMechanism;
    SparkMAXMechanism angleMechanism;
    int drivingCANId;
    int turningCANId;
    double chassisAngularOffset;

    SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModuleIOMAX(int index) {

        switch (index) {
            case 0: {
                drivingCANId = 10;
                turningCANId = 11;
                chassisAngularOffset = DriveConstants.kFrontLeftChassisAngularOffset;
                break;
            } case 1: {
                drivingCANId = 12;
                turningCANId = 13;
                chassisAngularOffset = DriveConstants.kFrontRightChassisAngularOffset;
                break;
            } case 2: {
                drivingCANId = 16;
                turningCANId = 17;
                chassisAngularOffset = DriveConstants.kBackLeftChassisAngularOffset;
                break;
            } case 3: {
                drivingCANId = 15;
                turningCANId = 14;
                chassisAngularOffset = DriveConstants.kBackRightChassisAngularOffset;
                break;
            } default: {
                throw new InvalidParameterException("Unquantified swerve module Requested");
            }

        }


        driveMechanism = new SparkMAXMechanism(drivingCANId);
        angleMechanism = new SparkMAXMechanism(turningCANId);
        driveMechanism.restoreFactoryDefault();
        angleMechanism.restoreFactoryDefault();

        driveMechanism.config_RelativePositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        driveMechanism.config_RelativeVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        driveMechanism.config_AbsoluteMode(false);
        driveMechanism.config_IdleMode(GenericIdleMode.kBrake);
        driveMechanism.config_kP(ModuleConstants.kDrivingP);
        driveMechanism.config_kI(ModuleConstants.kDrivingI);
        driveMechanism.config_kD(ModuleConstants.kDrivingD);
        driveMechanism.config_kV(ModuleConstants.kDrivingFF);

        driveMechanism.config_CurrentLimit(40);

        angleMechanism.config_kP(ModuleConstants.kTurningP);
        angleMechanism.config_kI(ModuleConstants.kTurningI);
        angleMechanism.config_kD(ModuleConstants.kTurningD);
        angleMechanism.config_kV(ModuleConstants.kDrivingFF);

        angleMechanism.config_CurrentLimit(20);

        angleMechanism.getPID().setPositionPIDWrappingEnabled(true);
        angleMechanism.getPID().setPositionPIDWrappingMinInput(0);
        angleMechanism.getPID().setPositionPIDWrappingMaxInput(1);

        angleMechanism.config_AbsolutePositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        angleMechanism.config_AbsoluteVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        angleMechanism.config_AbsoluteMode(true);
        angleMechanism.config_IdleMode(GenericIdleMode.kCoast);
    }

    @Override
    public void setSpeed(double speed) {
        driveMechanism.setBasicVelocity(speed);
    }
    @Override
    public void setAngle(double angle) {
        angleMechanism.setRacePIDPosition(angle + chassisAngularOffset);
    }

    @Override
    public SwerveModuleState optimize(SwerveModuleState swerveModuleState) {
        return SwerveModuleState.optimize(swerveModuleState, new Rotation2d(inputs.turnAbsolutePosition));
    }

    @Override
    public void stopDrive() {
        driveMechanism.stop();
    }

    @Override
    public void stopAngle() {
        driveMechanism.stop();
    }

    @Override
    public void tuneDrive() {
        driveMechanism.runPIDtuner();
    }
    @Override
    public void tuneAngle() {
        angleMechanism.runPIDtuner();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveMechanism.getPosition();
        inputs.driveVelocityMetersPerSec = driveMechanism.getVelocity();
        inputs.driveAppliedVolts = driveMechanism.getSupplyVoltage();
        inputs.driveCurrentAmps = driveMechanism.getAppliedCurrent();
        inputs.driveTemperatureCelsius = driveMechanism.getTemperature();

        inputs.turnAbsolutePosition = Units.degreesToRadians(angleMechanism.getAbsoluteSensorPosition() - chassisAngularOffset);
        inputs.turnVelocityRadPerSec = Units.degreesToRadians(angleMechanism.getAbsoluteSensorPosition());
        inputs.turnAppliedVolts = angleMechanism.getSupplyVoltage();
        inputs.turnCurrentAmps = angleMechanism.getAppliedCurrent();
        inputs.turnTemperatureCelsius = angleMechanism.getTemperature();
    }

    @Override
    public SwerveModuleIOInputs getInputs() {
        return inputs;
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
    }

}
