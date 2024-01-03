package frc.robot.libs.BaseMechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.libs.DiagnosticTable;

import java.util.function.DoubleConsumer;

public class SimMotorMechanism implements BaseMechanism{

    DCMotorSim motorSim;

    double POSITION_CONVERSION_FACTOR = 1;
    double VELOCITY_CONVERSION_FACTOR = 1;
    double POSITION_ERROR_TOLERANCE = 1;
    double ABSOLUTE_POSITION_CONVERSION_FACTOR = 1;
    double ABSOLUTE_VELOCITY_CONVERSION_FACTOR = 1;
    double RELATIVE_POSITION_CONVERSION_FACTOR = 1;
    double RELATIVE_VELOCITY_CONVERSION_FACTOR = 1;
    double _maxPosition = Double.MAX_VALUE;
    double _minPosition = Double.MIN_VALUE;

    boolean ABSOLUTE_MODE = false;

    DiagnosticTable dtab = new DiagnosticTable("MotorSim"+getClass());

    double _kP = 0;
    double _kI = 0;
    double _kIz = 0;
    double _kD = 0;
    double _kS = 0;
    double _kV = 0;
    double _kG = 0;
    double _kA = 0;
    double _kMinOutput = 0;
    double _kMaxOutput = 0;
    double _kMaxVelocity;
    double _kMinVelocity;
    double _kMaxAcceleration;
    boolean _toTrack = false;
    boolean _HasPIDTunerRun = false;
    boolean _velorpos = false;
    double _targPos = 0;

    ProfiledPIDController controlPID = new ProfiledPIDController(
            0,0,0,
            new TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE)
    );
    ProfiledPIDController VcontrolPID = new ProfiledPIDController(
            0,0,0,
            new TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE)
    );
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0,0);
    SimpleMotorFeedforward controlFF = new SimpleMotorFeedforward(0,0,0);

    double holdPosition;
    public SimMotorMechanism(DCMotorSim simMotor) {
        motorSim = simMotor;
    }

    public void updateSim() {
        motorSim.update(0.02);
    }

    @Override
    public int getMotorID() {
        return -1;
    }

    @Override
    public void setRawPercentOutput(double pct) {
        motorSim.setInputVoltage(12*pct);

    }

    @Override
    public void setThrottle(double pct) {
        setRawPercentOutput(pct);

    }

    @Override
    public void setVoltage(double volts) {
        motorSim.setInputVoltage(volts);

    }

    @Override
    public void setVelocity(double speed) {
        setBasicVelocity(speed);

    }

    @Override
    public void setBasicVelocity(double speed) {
        setVoltage(
                VcontrolPID.calculate(getVelocity(), speed)/VELOCITY_CONVERSION_FACTOR +
                controlFF.calculate(speed)/VELOCITY_CONVERSION_FACTOR
        );

    }

    @Override
    public double getVelocity() {
        return motorSim.getAngularVelocityRPM()*VELOCITY_CONVERSION_FACTOR;
    }

    @Override
    public void setPosition(double position) {
        setRacePIDPosition(position);
    }

    @Override
    public void setRacePIDPosition(double position) {
        motorSim.setInputVoltage(
                controlPID.calculate(getPosition(), constrainSetpoint(position))/POSITION_CONVERSION_FACTOR
        );

    }

    @Override
    public void setSmartPosition(double position) {
        setRacePIDPosition(position);
    }

    @Override
    public double getHoldPosition() {
        return holdPosition;
    }

    @Override
    public void setHoldPosition(double holdPosition) {
        this.holdPosition = holdPosition;
    }

    @Override
    public double getPosition() {
        return motorSim.getAngularPositionRotations()*POSITION_CONVERSION_FACTOR;
    }

    @Override
    public double getSupplyVoltage() {
        return 0;
    }

    @Override
    public double getAppliedCurrent() {
        return motorSim.getCurrentDrawAmps();
    }

    @Override
    public double getTemperature() {
        return 0;
    }

    @Override
    public double renderArbFF() {
        return Math.sin(getPosition())*_kG;
    }

    @Override
    public double getIntegratedSensorPosition() {
        return getPosition();
    }

    @Override
    public double getAbsoluteSensorPosition() {
        return getPosition();
    }

    @Override
    public double getSelectedSensorPosition() {
        return getPosition();
    }

    @Override
    public double getSelectedSensorVelocity() {
        return getVelocity();
    }

    @Override
    public double constrainSetpoint(double sp) {
        return MathUtil.clamp(sp, _minPosition, _maxPosition);
    }
    @Override
    public boolean isAtPoint(double point) {
        return isAtPoint(point, 1);
    }

    @Override
    public boolean isAtPoint(double point, double tolerance) {
        point = constrainSetpoint(point);
        double error = point-getSelectedSensorPosition();

        return Math.abs(error)<tolerance;
    }

    @Override
    public void config_PointCheckErrorTolerance(double tolerance) {
        POSITION_ERROR_TOLERANCE = tolerance;
    }

    @Override
    public void config_PositionConversionFactor(double factor) {
        POSITION_CONVERSION_FACTOR = factor;
    }

    @Override
    public double get_PositionConversionFactor() {
        return POSITION_CONVERSION_FACTOR;
    }

    public void config_RelativePositionConversionFactor(double factor) {
        RELATIVE_POSITION_CONVERSION_FACTOR = factor;
    }

    @Override
    public double get_RelativePositionConversionFactor() {
        return RELATIVE_POSITION_CONVERSION_FACTOR;
    }

    @Override
    public void config_AbsolutePositionConversionFactor(double factor) {
        ABSOLUTE_POSITION_CONVERSION_FACTOR = factor;
    }

    @Override
    public double get_AbsolutePositionConversionFactor() {
        return ABSOLUTE_POSITION_CONVERSION_FACTOR;
    }

    @Override
    public void config_VelocityConversionFactor(double factor) {
        VELOCITY_CONVERSION_FACTOR = factor;
    }

    @Override
    public double get_VelocityConversionFactor() {
        return VELOCITY_CONVERSION_FACTOR;
    }

    @Override
    public void config_RelativeVelocityConversionFactor(double factor) {
        RELATIVE_VELOCITY_CONVERSION_FACTOR = factor;
    }

    @Override
    public double get_RelativeVelocityConversionFactor() {
        return RELATIVE_VELOCITY_CONVERSION_FACTOR;
    }

    @Override
    public void config_AbsoluteVelocityConversionFactor(double factor) {
        ABSOLUTE_VELOCITY_CONVERSION_FACTOR = factor;
    }

    @Override
    public double get_AbsoluteVelocityConversionFactor() {
        return ABSOLUTE_VELOCITY_CONVERSION_FACTOR;
    }


    @Override
    public void config_IdleMode(GenericIdleMode idleMode) {

    }

    @Override
    public void config_SmartMotionConstraints(TrapezoidProfile.Constraints constraints) {
        controlPID.setConstraints(constraints);
    }

    @Override
    public void config_SmartMotionCruiseVelocity(double vel) {
        _kMaxVelocity = vel;
        config_SmartMotionConstraints(
                new TrapezoidProfile.Constraints(
                        _kMaxVelocity,
                        _kMinVelocity
                )
        );

    }

    @Override
    public void config_SmartMotionMaxAcceleration(double vel_squared) {
        _kMaxVelocity = vel_squared;
        config_SmartMotionConstraints(
                new TrapezoidProfile.Constraints(
                        _kMaxVelocity,
                        _kMinVelocity
                )
        );
    }

    @Override
    public void config_kP(double kP) {
        _kP = kP;
        controlPID.setP(kP);
    }

    @Override
    public void config_kI(double kI) {
        _kI = kI;
        controlPID.setI(kI);
    }

    @Override
    public void config_kIz(double kIz) {

    }

    @Override
    public void config_kD(double kD) {
        _kD = kD;
        controlPID.setD(kD);

    }

    @Override
    public void config_kV(double kV) {
        _kV = kV;
        controlFF = new SimpleMotorFeedforward(_kS, _kV, _kA);
    }

    @Override
    public void config_kS(double kS) {
        _kS = kS;
        controlFF = new SimpleMotorFeedforward(_kS, _kV, _kA);
    }

    @Override
    public void config_kG(double kG) {
        _kG = kG;
    }

    @Override
    public void config_kA(double kA) {
        _kA = kA;
        controlFF = new SimpleMotorFeedforward(_kS, _kV, _kA);
    }

    @Override
    public void config_PIDOutputRange(double lowerbound, double upperbound) {

    }

    @Override
    public void config_PositionMode(ControlModes.PositionMode positionMode) {

    }

    @Override
    public void config_VelocityMode(ControlModes.VelocityMode velocityMode) {

    }

    @Override
    public void config_PIDMethod(ControlModes.PIDMethod PIDMethod) {

    }

    @Override
    public void config_MinSetpoint(double minSet) {
        _minPosition = minSet;

    }

    @Override
    public void config_MaxSetpoint(double maxSet) {
        _maxPosition = maxSet;

    }

    @Override
    public void config_SetpointRange(double minSet, double maxSet) {
        config_MinSetpoint(minSet);
        config_MaxSetpoint(maxSet);

    }

    @Override
    public void config_CurrentLimit(double maxAmps) {

    }

    @Override
    public void config_ClosedLoopRampRate(double seconds) {

    }

    @Override
    public void config_OpenLoopRampRate(double seconds) {

    }

    @Override
    public void setIntegratedSensorPosition(double position) {

    }

    @Override
    public void config_AbsoluteMode(boolean absoluteMode) {
        ABSOLUTE_MODE = absoluteMode;
        if(absoluteMode) {
            POSITION_CONVERSION_FACTOR = ABSOLUTE_POSITION_CONVERSION_FACTOR;
            VELOCITY_CONVERSION_FACTOR = ABSOLUTE_VELOCITY_CONVERSION_FACTOR;
        } else {
            POSITION_CONVERSION_FACTOR = RELATIVE_POSITION_CONVERSION_FACTOR;
            VELOCITY_CONVERSION_FACTOR = RELATIVE_VELOCITY_CONVERSION_FACTOR;
        }
    }

    @Override
    public void saveConfigs() {

    }

    @Override
    public void restoreFactoryDefault() {

    }

    @Override
    public double getNumberNT(String key, double defaultValue) {
        return dtab.getNumber(key,defaultValue);
    }

    @Override
    public void putNumberNT(String key, double value) {
        dtab.putNumber(key, value);
    }

    @Override
    public boolean getBooleanNT(String key, boolean defaultValue) {
        return dtab.getBoolean(key, defaultValue);
    }

    @Override
    public void putBooleanNT(String key, boolean value) {
        dtab.putBoolean(key, value);

    }

    @Override
    public void runPIDtuner() {
        if(!_HasPIDTunerRun) {
            putBooleanNT("toTrack", false);
            putBooleanNT("Vel_or_pos", false);
            putNumberNT("targPos", 0.0);


            putNumberNT("P Gain", _kP);
            putNumberNT("I Gain", _kI);
            putNumberNT("D Gain", _kD);
            putNumberNT("I Zone", _kIz);
            putNumberNT("Feed Forward (Ordinary)", _kV);
            putNumberNT("Feed Forward (Static Friction Compensation)", _kS);
            putNumberNT("Feed Forward (Gravity Compensation)", _kG);
            putNumberNT("Feed Forward (Acceleration Compensation)", _kA);


            putNumberNT("Max Output", 0);
            putNumberNT("Min Output", 0);
            putNumberNT("Max Velocity", 0);
            putNumberNT("Min Velocity", 0);
            putNumberNT("Max Acceleration", 0);
            putNumberNT("Allowed Closed Loop Error", 0);
            _HasPIDTunerRun = true;
        }

        boolean toTrack = getBooleanNT("toTrack", false);
        boolean vel_or_pos = getBooleanNT("Vel_or_pos", false);
        double trackPos = getNumberNT("targPos", 0.0);


        double p = getNumberNT("P Gain", 0);
        double i = getNumberNT("I Gain", 0);
        double d = getNumberNT("D Gain", 0);
        double iz = getNumberNT("I Zone", 0);
        double kV = getNumberNT("Feed Forward (Ordinary)", 0);
        double kS = getNumberNT("Feed Forward (Static Friction Compensation)", 0);
        double kG = getNumberNT("Feed Forward (Gravity Compensation)", 0);
        double kA = getNumberNT("Feed Forward (Acceleration Compensation)", 0);


        double max = getNumberNT("Max Output", 0);
        double min = getNumberNT("Min Output", 0);
        double maxV = getNumberNT("Max Velocity", 0);
        double minV = getNumberNT("Min Velocity", 0);
        double maxA = getNumberNT("Max Acceleration", 0);
        double allE = getNumberNT("Allowed Closed Loop Error", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if(trackPos != _targPos) { _targPos = trackPos; }
        if(toTrack != _toTrack) { _toTrack = toTrack; }
        if(vel_or_pos != _velorpos) { _velorpos = vel_or_pos; }


        if((p != _kP)) { config_kP(p); }
        if((i != _kI)) { config_kI(i); }
        if((d != _kD)) { config_kD(d); }
        if((iz != _kIz)) { config_kIz(iz); }
        if((kV != _kV)) { config_kV(kV); }
        if((kS != _kS)) { config_kS(kS); }
        if((kS != _kG)) { config_kG(kG); }
        if((kS != _kA)) { config_kA(kA); }

        if((max != _kMaxOutput) || (min != _kMinOutput)) {

            _kMinOutput = min; _kMaxOutput = max;
        }
        if((maxV != _kMaxVelocity)) { config_SmartMotionCruiseVelocity(maxV); _kMaxVelocity = maxV; }
        if((maxA != _kMaxAcceleration)) { config_SmartMotionMaxAcceleration(maxA); _kMaxAcceleration = maxA; }
        //if((allE != _kMaxClosedLoopError)) { config_MaxAllowedClosedLoopError(allE); _kMaxClosedLoopError = allE; }


        if(_toTrack) {
            if(!vel_or_pos) {
                setSmartPosition(_targPos);
                putNumberNT("TargError", getPosition()-_targPos);
            } else {
                setBasicVelocity(_targPos);
                putNumberNT("TargError", getVelocity()-_targPos);
            }
        }

    }

    private void constrainOutput(double lb, double ub){
        _kMinOutput = lb;
        _kMaxOutput = ub;

    }
}
