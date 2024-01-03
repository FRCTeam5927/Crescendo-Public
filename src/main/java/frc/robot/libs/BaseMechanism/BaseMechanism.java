package frc.robot.libs.BaseMechanism;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public interface BaseMechanism {
    NetworkTableInstance _netinst = NetworkTableInstance.getDefault();

    Timer holdChecker = new Timer();
    

    int getMotorID();
    default void stop() {
        setRawPercentOutput(0.0);
    };
    void setRawPercentOutput(double pct);
    void setThrottle(double pct);
    void setVoltage(double volts);
    void setVelocity(double speed);
    void setBasicVelocity(double speed);
    double getVelocity();
    void setPosition(double position);
    void setRacePIDPosition(double position);
    void setSmartPosition(double position);
    double getHoldPosition();
    void setHoldPosition(double holdPosition);
    default void powerBrake() {
        boolean firstLoop = holdChecker.get() > 0.1;
        if(firstLoop) {
            setHoldPosition(getPosition());
        }
        setPosition(getHoldPosition());
        holdChecker.reset();
        holdChecker.start();


    };
    double getPosition();
    double getSupplyVoltage();
    double getAppliedCurrent();
    double getTemperature();
    double renderArbFF();
    double getIntegratedSensorPosition();
    double getAbsoluteSensorPosition();
    double getSelectedSensorPosition();
    double getSelectedSensorVelocity();
    double constrainSetpoint(double sp);
    boolean isAtPoint(double point);
    boolean isAtPoint(double point, double tolerance);
    void config_PointCheckErrorTolerance(double tolerance);
    void config_PositionConversionFactor(double factor);
    double get_PositionConversionFactor();
    void config_RelativePositionConversionFactor(double factor);
    double get_RelativePositionConversionFactor();
    void config_AbsolutePositionConversionFactor(double factor);
    double get_AbsolutePositionConversionFactor();
    void config_VelocityConversionFactor(double factor);
    double get_VelocityConversionFactor();
    void config_RelativeVelocityConversionFactor(double factor);
    double get_RelativeVelocityConversionFactor();

    void config_AbsoluteVelocityConversionFactor(double factor);
    double get_AbsoluteVelocityConversionFactor();

    void config_IdleMode(GenericIdleMode idleMode);
    void config_SmartMotionConstraints(TrapezoidProfile.Constraints constraints);
    void config_SmartMotionCruiseVelocity(double vel);
    void config_SmartMotionMaxAcceleration(double vel_squared);
    void config_kP(double kP);
    void config_kI(double kI);
    void config_kIz(double kIz);
    void config_kD(double kD);
    void config_kV(double kV);
    void config_kS(double kS);
    void config_kG(double kS);

    /**
     * please note that kA may only be implemented in the modes where feedForward is rendered on the RIO:
     *  SoftwarePIDF, SoftwareFF, HybridFF, onControllerPIDSoftwareFF, and SoftwarePIDHybridFF
     * @param kA
     */
    void config_kA(double kA);
    void config_PIDOutputRange(double lowerbound, double upperbound);
    void config_PositionMode(ControlModes.PositionMode positionMode);
    void config_VelocityMode(ControlModes.VelocityMode velocityMode);
    void config_PIDMethod(ControlModes.PIDMethod PIDMethod);
    void config_MinSetpoint(double minSet);
    void config_MaxSetpoint(double maxSet);
    void config_SetpointRange(double minSet, double maxSet);
    void config_CurrentLimit(double maxAmps);
    void config_ClosedLoopRampRate(double seconds);
    void config_OpenLoopRampRate(double seconds);

    void setIntegratedSensorPosition(double position);
    void config_AbsoluteMode(boolean absoluteMode);
    void saveConfigs();
    void restoreFactoryDefault();
    double getNumberNT(String key, double defaultValue);
    void putNumberNT(String key, double value);
    boolean getBooleanNT(String key, boolean defaultValue);
    void  putBooleanNT(String key, boolean value);

    void runPIDtuner();
}
