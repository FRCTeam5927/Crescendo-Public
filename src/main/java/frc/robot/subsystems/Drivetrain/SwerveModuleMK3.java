//package frc.robot.subsystems.Drivetrain;
//
//import com.ctre.phoenix6.configs.*;
//import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.Drivetrain.MaxSwerveConstants;
//import frc.robot.libs.BaseMechanism.GenericIdleMode;
//import frc.robot.libs.BaseMechanism.TalonFXMechanism;
//import frc.robot.libs.interfaces.ISwerveModule;
//
//public class SwerveModuleMK3 extends SubsystemBase implements ISwerveModule {
//    TalonFXMechanism _driveMechanism;
//    TalonFXMechanism _angleMechanism;
//    Translation2d modulePosition;
//
//    CANcoder _canCoder;
//    public SwerveModuleMK3(int driveID, int steerID, int encoderID, double moduleOffset, Translation2d modulePosition) {
//        this.modulePosition = modulePosition;
//        _canCoder = new CANcoder(encoderID);
//        _angleMechanism = new TalonFXMechanism(steerID);
//
//        _driveMechanism = new TalonFXMechanism(driveID);
//
//        _angleMechanism.restoreFactoryDefault();
//        _angleMechanism.restoreFactoryDefault();
//
//        _driveMechanism.config_RelativePositionConversionFactor(MaxSwerveConstants.WHEEL_EFFECTIVE_CIRCUMFERENCE);
//        _driveMechanism.config_RelativeVelocityConversionFactor(MaxSwerveConstants.WHEEL_EFFECTIVE_CIRCUMFERENCE);
//        _driveMechanism.config_AbsoluteMode(false);
//        _driveMechanism.config_IdleMode(GenericIdleMode.kBrake);
//        _driveMechanism.config_kP(0.23);
//        _driveMechanism.config_kI(0.0);
//        _driveMechanism.config_kD(0.0);
//        _driveMechanism.config_kV(0.106);
//        _driveMechanism.config_kS(0.14);
//
//
//        _driveMechanism.config_CurrentLimit(35);
//
//
//        _angleMechanism.config_kP(40);
//        _angleMechanism.config_kP(40.402);
//        _angleMechanism.config_kI(09.0);
//        _angleMechanism.config_kD(0.0);
//        _angleMechanism.config_kV(0.0);
//        _angleMechanism.config_kS(0.10);
//
//        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
//        closedLoopGeneralConfigs.ContinuousWrap = true;
//        _angleMechanism.getMotor().getConfigurator().apply(closedLoopGeneralConfigs);
//
//        double factor = 360;
//
//        _angleMechanism.config_AbsolutePositionConversionFactor(factor);
//        _angleMechanism.config_AbsoluteVelocityConversionFactor(factor);
//        _angleMechanism.setRemoteSensorCanID(_canCoder.getDeviceID());
//
//
//        _angleMechanism.config_RelativePositionConversionFactor(1/12.8*360);
//        _angleMechanism.config_RelativeVelocityConversionFactor(1/12.8*360);
//
//        _angleMechanism.config_AbsoluteMode(true);
//        _angleMechanism.config_IdleMode(GenericIdleMode.kCoast);
//
//        ClosedLoopRampsConfigs rampsConfigs = new ClosedLoopRampsConfigs();
//        rampsConfigs.VoltageClosedLoopRampPeriod = 0.07;
//        rampsConfigs.TorqueClosedLoopRampPeriod = 0.1;
//
//        _angleMechanism.getMotor().getConfigurator().apply(rampsConfigs);
//
//        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
//        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
//        magnetSensorConfigs.MagnetOffset = moduleOffset;
//        _canCoder.getConfigurator().apply(magnetSensorConfigs);
////        _desiredState.angle = new Rotation2d(getAbsoluteAngleRadians());
//    }
//
//    @Override
//    public void periodic() {
//
//    }
//
//    @Override
//    public void setDesiredState(SwerveModuleState desiredState) {
//        double[] optimizedState = optimize(desiredState);
//
//        _driveMechanism.setBasicVelocity(optimizedState[1]);
//
//        if(Math.abs(optimizedState[1])<0.01) {
//            _angleMechanism.setRacePIDPosition(optimizedState[0]);
//        }
//    }
//
//    /**
//     *
//     * @param angle MUST be within 0-360, otherwise the system is VERY unhappy.
//     */
//    @Override
//    public void setModuleAngle(double angle) {
//        _driveMechanism.stop();
//        _angleMechanism.setRacePIDPosition(angle);
//    }
//
//    @Override
//    public void pointToCenter() {
//        setModuleAngle(modulePosition.getAngle().getDegrees());
//    }
//
////    @Override
////    public void pointToCenter() {
////        setModuleAngle(modulePosition.getAngle().getDegrees());
////    }
//
//    @Override
//    public SwerveModuleState getModuleState() {
//        return new SwerveModuleState(
//                _driveMechanism.getVelocity(),
//                getModulePosition().angle
//        );
//    }
//
//    @Override
//    public SwerveModulePosition getModulePosition() {
//        return new SwerveModulePosition(
//                _driveMechanism.getPosition(),
//                Rotation2d.fromRadians(_canCoder.getAbsolutePosition().getValue()*2*Math.PI)
//        );
//    }
//
//    @Override
//    public double getDriveTemperature() {
//        return _driveMechanism.getMotor().getDeviceTemp().getValue();
//    }
//
//    @Override
//    public double getAngleTemperature() {
//        return _angleMechanism.getMotor().getDeviceTemp().getValue();
//    }
//
//    public TalonFXMechanism get_driveMechanism() {
//        return _driveMechanism;
//    }
//    public TalonFXMechanism get_angleMechanism() {
//        return _angleMechanism;
//    }
//    private double[] optimize(SwerveModuleState desiredState){
//        double desiredDegrees = desiredState.angle.getDegrees();
//        double desiredMPS = desiredState.speedMetersPerSecond;
//
//
//        double optimizedDegrees;
//        double optimizedMPS;
//        var delta = getModulePosition().angle.getDegrees()-desiredDegrees;
//        if (Math.abs(delta) > 90.0) {
//            if(desiredDegrees<180) {
//                optimizedDegrees = desiredDegrees + 180;
//                optimizedMPS = -desiredMPS;
//            } else {
//                optimizedDegrees = desiredDegrees - 180;
//                optimizedMPS = -desiredMPS;
//            }
//        } else {
//            optimizedDegrees = desiredDegrees;
//            optimizedMPS = desiredMPS;
//        }
//
//        return new double[] {
//                optimizedDegrees,optimizedMPS
//        };
//    }
//
//}
