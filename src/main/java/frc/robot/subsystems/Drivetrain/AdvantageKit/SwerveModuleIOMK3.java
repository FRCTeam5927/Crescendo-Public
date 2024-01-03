package frc.robot.subsystems.Drivetrain.AdvantageKit;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain.MaxSwerveConstants;
import frc.robot.libs.BaseMechanism.GenericIdleMode;
import frc.robot.libs.BaseMechanism.TalonFXMechanism;

import java.security.InvalidParameterException;

public class SwerveModuleIOMK3 extends SubsystemBase implements SwerveModuleIO {
    TalonFXMechanism driveMechanism;
    TalonFXMechanism angleMechanism;
    CANcoder canCoder;

    SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

    public SwerveModuleIOMK3(int index) {


        int drivingCANId;
        int turningCANId;
        double chassisAngularOffset;
        int encoderCANId;

        switch (index) {
            case 0: {
                drivingCANId = 5;
                turningCANId = 6;
                chassisAngularOffset = -0.68872070;
                encoderCANId = 12;
                break;
            } case 1: {
                drivingCANId = 7;
                turningCANId = 8;
                chassisAngularOffset = -0.23632812;
                encoderCANId = 14;
                break;
            } case 2: {
                drivingCANId = 3;
                turningCANId = 4;
                chassisAngularOffset = -0.93774414;
                encoderCANId = 11;
                break;
            } case 3: {
                drivingCANId = 9;
                turningCANId = 10;
                chassisAngularOffset = -0.99536133;
                encoderCANId = 13;
                break;
            } default: {
                throw new InvalidParameterException("Unquantified swerve module Requested");
            }

        }


        driveMechanism = new TalonFXMechanism(drivingCANId);
        angleMechanism = new TalonFXMechanism(turningCANId);
        driveMechanism.restoreFactoryDefault();
        angleMechanism.restoreFactoryDefault();
        canCoder = new CANcoder(encoderCANId);

        driveMechanism.config_RelativePositionConversionFactor(MaxSwerveConstants.WHEEL_EFFECTIVE_CIRCUMFERENCE);
        driveMechanism.config_RelativeVelocityConversionFactor(MaxSwerveConstants.WHEEL_EFFECTIVE_CIRCUMFERENCE);
        driveMechanism.config_AbsoluteMode(false);
        driveMechanism.config_IdleMode(GenericIdleMode.kBrake);
        driveMechanism.config_kP(0.23);
        driveMechanism.config_kI(0.0);
        driveMechanism.config_kD(0.0);
        driveMechanism.config_kV(0.106);
        driveMechanism.config_kS(0.14);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentThreshold = 35;
        currentLimitsConfigs.SupplyCurrentLimit = 40;
        currentLimitsConfigs.SupplyTimeThreshold = 100;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        driveMechanism.getMotor().getConfigurator().apply(currentLimitsConfigs);

        angleMechanism.config_kP(40.402);
        angleMechanism.config_kI(09.0);
        angleMechanism.config_kD(0.0);
        angleMechanism.config_kV(0.0);
        angleMechanism.config_kS(0.10);

        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        closedLoopGeneralConfigs.ContinuousWrap = true;
        angleMechanism.getMotor().getConfigurator().apply(closedLoopGeneralConfigs);

        double factor = 2*Math.PI;
        angleMechanism.config_AbsolutePositionConversionFactor(factor);
        angleMechanism.config_AbsoluteVelocityConversionFactor(factor);
        angleMechanism.setRemoteSensorCanID(canCoder.getDeviceID());

//        angleMechanism.config_RelativePositionConversionFactor(1/12.8*360);
//        angleMechanism.config_RelativeVelocityConversionFactor(1/12.8*360);

        angleMechanism.config_AbsoluteMode(true);
        angleMechanism.config_IdleMode(GenericIdleMode.kCoast);

        ClosedLoopRampsConfigs rampsConfigs = new ClosedLoopRampsConfigs();
        rampsConfigs.VoltageClosedLoopRampPeriod = 0.07;
        rampsConfigs.TorqueClosedLoopRampPeriod = 0.1;

        angleMechanism.getMotor().getConfigurator().apply(rampsConfigs);

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.MagnetOffset = chassisAngularOffset;
        canCoder.getConfigurator().apply(magnetSensorConfigs);


    }

    @Override
    public void setSpeed(double speed) {
//        driveMechanism.setBasicVelocity(speed);
    }
    @Override
    public void setAngle(double angle) {
//        angleMechanism.setRacePIDPosition(angle);
    }

    @Override
    public SwerveModuleState optimize(SwerveModuleState desiredState) {
        SwerveModuleState correctedState = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle.plus(new Rotation2d(Math.PI))
        );


        double desiredDegrees = correctedState.angle.getDegrees();
        double desiredMPS = correctedState.speedMetersPerSecond;


        double optimizedDegrees;
        double optimizedMPS;
        var delta = Units.radiansToDegrees(inputs.turnAbsolutePosition) - desiredDegrees;
        if (Math.abs(delta) > 90.0) {
            if (desiredDegrees < 180) {
                optimizedDegrees = desiredDegrees + 180;
                optimizedMPS = -desiredMPS;
            } else {
                optimizedDegrees = desiredDegrees - 180;
                optimizedMPS = -desiredMPS;
            }
        } else {
            optimizedDegrees = desiredDegrees;
            optimizedMPS = desiredMPS;
        }

        return new SwerveModuleState(optimizedMPS, Rotation2d.fromDegrees(optimizedDegrees));

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

        inputs.turnAbsolutePosition = canCoder.getAbsolutePosition().getValue()*2*Math.PI;
        inputs.turnVelocityRadPerSec = Units.degreesToRadians(angleMechanism.getVelocity());
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
