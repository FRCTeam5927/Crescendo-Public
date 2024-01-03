package frc.robot.libs.BaseMechanism;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.SparkMaxPIDController.*;
import frc.robot.Constants.DebugConstants;

import java.util.function.DoubleConsumer;

import static com.revrobotics.CANSparkMax.ControlType.kSmartMotion;

public class SparkMAXMechanism implements BaseMechanism, Subsystem {
    CANSparkMax _motor;
    SparkMaxPIDController _pidcontroller;
    protected NetworkTable _tunerTab;

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

    /**
     * SmartMotion Constraints
     */
    double _kMaxVelocity;
    double _kMinVelocity;
    double _kMaxAcceleration;
    double _kMaxClosedLoopError;
    boolean _HasPIDTunerRun = false;

    double _maxPosition = Double.MAX_VALUE;
    double _minPosition = Double.MIN_VALUE;

    boolean _velorpos = false;
    boolean _toTrack = false;
    double _targPos = 0.0;

    RelativeEncoder _integratedEncoder;

    protected AbsoluteEncoder _absoluteEncoder;
    DoubleConsumer _positionSetter = this::setSmartPosition;
    DoubleConsumer _velocitySetter = this::setBasicVelocity;

    double POSITION_CONVERSION_FACTOR = 1;
    double VELOCITY_CONVERSION_FACTOR = 1;
    double POSITION_ERROR_TOLERANCE = 1;
    double ABSOLUTE_POSITION_CONVERSION_FACTOR = 1;
    double ABSOLUTE_VELOCITY_CONVERSION_FACTOR = 1;
    double RELATIVE_POSITION_CONVERSION_FACTOR = 1;
    double RELATIVE_VELOCITY_CONVERSION_FACTOR = 1;

    boolean ABSOLUTE_MODE = false;


    double holdPosition;

    public SparkMAXMechanism(int canID) {
        this(canID, CANSparkMaxLowLevel.MotorType.kBrushless);
    }
    public SparkMAXMechanism(int canID, CANSparkMaxLowLevel.MotorType motorType) {
        _motor = new CANSparkMax(canID, motorType);
        _pidcontroller = _motor.getPIDController();
        _integratedEncoder = _motor.getEncoder();
        _absoluteEncoder = _motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        CommandScheduler.getInstance().registerSubsystem(this);

        _tunerTab = _netinst.getTable("SparkMaxMechanism_ID_"+canID);
    }

    @Override
    public int getMotorID() {
        return _motor.getDeviceId();
    }

    @Override
    public void setRawPercentOutput(double pct) {
        _motor.set(pct);
    }

    @Override
    public void setThrottle(double pct) {
        _pidcontroller.setReference(pct, CANSparkMax.ControlType.kDutyCycle, 0, 
                renderArbFF(), ArbFFUnits.kVoltage);
    }

    @Override
    public void setVoltage(double volts) {
        _pidcontroller.setReference(volts, CANSparkMax.ControlType.kVoltage, 0,
                renderArbFF(), ArbFFUnits.kVoltage);
    }

    @Override
    public void setVelocity(double speed) {
        _velocitySetter.accept(speed);
    }

    @Override
    public void setBasicVelocity(double speed) {
        _pidcontroller.setReference(speed/VELOCITY_CONVERSION_FACTOR, CANSparkMax.ControlType.kVelocity, 0,
                renderArbFF(), ArbFFUnits.kVoltage);
    }

    @Override
    public double getVelocity() {
        return _integratedEncoder.getVelocity()*VELOCITY_CONVERSION_FACTOR;
    }

    @Override
    public void setPosition(double position) {
        _positionSetter.accept(position);
    }

    @Override
    public void setRacePIDPosition(double position) {
        position = constrainSetpoint(position);
        _pidcontroller.setReference(position/POSITION_CONVERSION_FACTOR, CANSparkMax.ControlType.kPosition, 0,
                renderArbFF(), ArbFFUnits.kVoltage);
    }

    @Override
    public void setSmartPosition(double position) {
        position = constrainSetpoint(position);
        _pidcontroller.setReference(position/POSITION_CONVERSION_FACTOR, kSmartMotion, 0,
                renderArbFF(), ArbFFUnits.kVoltage);
    }

    @Override
    public double getHoldPosition() {
        return this.holdPosition;
    };


    @Override
    public void setHoldPosition(double holdPosition) {
        this.holdPosition = holdPosition;
    };


    @Override
    public double getPosition() {
        return _integratedEncoder.getPosition()*POSITION_CONVERSION_FACTOR;
    }

    @Override
    public double getSupplyVoltage() {
        return _motor.getBusVoltage();
    }

    @Override
    public double getAppliedCurrent() {
        return _motor.getOutputCurrent();
    }

    @Override
    public double getTemperature() {
        return _motor.getMotorTemperature();
    }

    @Override
    public double renderArbFF() {
        double arbFF = 0.0;
//        arbFF += _kS;
        arbFF += _kG * Math.sin(Units.degreesToRadians(getPosition()));
        return arbFF;
    }

    @Override
    public double getIntegratedSensorPosition() {
        return _integratedEncoder.getPosition()*RELATIVE_POSITION_CONVERSION_FACTOR;
    }

    @Override
    public double getAbsoluteSensorPosition() {
        return _absoluteEncoder.getPosition()*ABSOLUTE_POSITION_CONVERSION_FACTOR;
    }

    @Override
    public double getSelectedSensorPosition() {
        double pos = 0;
        if(ABSOLUTE_MODE) {
            pos = _absoluteEncoder.getPosition()*ABSOLUTE_POSITION_CONVERSION_FACTOR;
        } else {
            pos = _integratedEncoder.getPosition()*RELATIVE_POSITION_CONVERSION_FACTOR;
        }
        return pos;
    }

    @Override
    public double getSelectedSensorVelocity() {
        double vel = 0;
        if(ABSOLUTE_MODE) {
            vel = _absoluteEncoder.getVelocity()*ABSOLUTE_VELOCITY_CONVERSION_FACTOR;
        } else {
            vel = _integratedEncoder.getVelocity()*RELATIVE_VELOCITY_CONVERSION_FACTOR;
        }
        return vel;
    }

    @Override
    public double constrainSetpoint(double sp) {
        return MathUtil.clamp(sp, _minPosition, _maxPosition);
    }

    @Override
    public boolean isAtPoint(double point) {
        return isAtPoint(point, POSITION_ERROR_TOLERANCE);
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

    @Override
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
        switch (idleMode) {
            case kCoast:
                _motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
                break;

            case kBrake:
                _motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
                break;

            default:
                System.out.println("INVALID IDLE MODE");
        }

    }

    @Override
    public void config_SmartMotionConstraints(TrapezoidProfile.Constraints constraints) {
//        _pidcontroller.setSmartMotionMaxAccel(constraints.maxAcceleration/VELOCITY_CONVERSION_FACTOR, 0);
//        _pidcontroller.setSmartMotionMaxVelocity(constraints.maxVelocity, 0);
//        _pidcontroller.setSmartMotionMinOutputVelocity(0.01, 0);
        config_SmartMotionCruiseVelocity(constraints.maxVelocity);
        config_SmartMotionMaxAcceleration(constraints.maxAcceleration);
    }

    @Override
    public void config_SmartMotionCruiseVelocity(double vel) {
        _pidcontroller.setSmartMotionMaxVelocity(vel/VELOCITY_CONVERSION_FACTOR, 0);
    }

    @Override
    public void config_SmartMotionMaxAcceleration(double vel_squared) {
        _pidcontroller.setSmartMotionMaxAccel(vel_squared/VELOCITY_CONVERSION_FACTOR, 0);
    }

    public void config_SmartMotionMinVelocity(double vel) {
        _pidcontroller.setSmartMotionMinOutputVelocity(vel/VELOCITY_CONVERSION_FACTOR, 0);
    }
    public void config_MaxAllowedClosedLoopError(double error) {
        _pidcontroller.setSmartMotionAllowedClosedLoopError(error/POSITION_CONVERSION_FACTOR, 0);
    }


    @Override
    public void config_kP(double kP) {
        _kP = kP;
        _pidcontroller.setP(kP);
    }

    @Override
    public void config_kI(double kI) {
        _kI = kI;
        _pidcontroller.setD(kI);
    }

    @Override
    public void config_kIz(double kIz) {
        _kIz = kIz;
        _pidcontroller.setIZone(kIz);
    }

    @Override
    public void config_kD(double kD) {
        _kD = kD;
        _pidcontroller.setD(kD);
    }

    @Override
    public void config_kV(double kV) {
        _kV = kV;
        _pidcontroller.setFF(kV);
    }

    @Override
    public void config_kS(double kS) {
        _kS = kS;
    }

    @Override
    public void config_kG(double kG) {
        _kG = kG;
    }

    @Override
    public void config_kA(double kA) {
        _kA = kA;
    }

    @Override
    public void config_PIDOutputRange(double lowerbound, double upperbound) {
        _pidcontroller.setOutputRange(lowerbound, upperbound);
    }

    @Override
    public void config_PositionMode(ControlModes.PositionMode positionMode) {
        switch (positionMode) {
            case RacePID:
                _positionSetter = this::setRacePIDPosition;
                break;
            case SmartMotion:
                _positionSetter = this::setSmartPosition;
                break;

        }


    }

    @Override
    public void config_VelocityMode(ControlModes.VelocityMode velocityMode) {
        switch (velocityMode) {
            case Basic:
                _velocitySetter = this::setBasicVelocity;
                break;

            default:
                System.out.println("INVALID VELOCITY TYPE");

        }
    }

    @Override
    public void config_PIDMethod(ControlModes.PIDMethod PIDMethod) {
//        switch (PIDMethod) {
//            case
//        }

    }

    @Override
    public void config_MinSetpoint(double minSet) {
        _minPosition = minSet;
    }

    @Override
    public void config_MaxSetpoint(double maxSet) {
        _maxPosition = maxSet;
    }

    public void config_SoftLimitMax(double pos) {
        _motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) (pos/POSITION_CONVERSION_FACTOR));
        _motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    }

    public void config_SoftLimitMin(double pos) {
        _motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) (pos/POSITION_CONVERSION_FACTOR));
        _motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    @Override
    public void config_SetpointRange(double minSet, double maxSet) {
        config_MinSetpoint(minSet);
        config_MaxSetpoint(maxSet);
    }

    @Override
    public void config_CurrentLimit(double maxAmps) {
        _motor.setSmartCurrentLimit((int) (maxAmps*1.1), (int) maxAmps,60);
//        _motor.setSecondaryCurrentLimit(maxAmps);
    }

    @Override
    public void config_ClosedLoopRampRate(double seconds) {
        _motor.setClosedLoopRampRate(seconds);

    }

    @Override
    public void config_OpenLoopRampRate(double seconds) {
        _motor.setOpenLoopRampRate(seconds);
    }

    @Override
    public void setIntegratedSensorPosition(double position) {
        _integratedEncoder.setPosition(position/RELATIVE_POSITION_CONVERSION_FACTOR);
    }

    @Override
    public void config_AbsoluteMode(boolean absoluteMode) {
        ABSOLUTE_MODE = absoluteMode;
        if(absoluteMode) {
            _pidcontroller.setFeedbackDevice(_absoluteEncoder);
            POSITION_CONVERSION_FACTOR = ABSOLUTE_POSITION_CONVERSION_FACTOR;
            VELOCITY_CONVERSION_FACTOR = ABSOLUTE_VELOCITY_CONVERSION_FACTOR;
        } else {
            _pidcontroller.setFeedbackDevice(_integratedEncoder);
            POSITION_CONVERSION_FACTOR = RELATIVE_POSITION_CONVERSION_FACTOR;
            VELOCITY_CONVERSION_FACTOR = RELATIVE_VELOCITY_CONVERSION_FACTOR;
        }
    }

    @Override
    public void saveConfigs() {
        _motor.burnFlash();
    }

    @Override
    public void restoreFactoryDefault() {
        _motor.restoreFactoryDefaults();
    }

    @Override
    public void putNumberNT(String name, double value) {
        _tunerTab.getEntry(name).setDouble(value);
    }

    @Override
    public double getNumberNT(String name, double defaultValue) {
        return _tunerTab.getEntry(name).getDouble(defaultValue);
    }

    @Override
    public void putBooleanNT(String name, boolean value) {
        _tunerTab.getEntry(name).setBoolean(value);
    }

    @Override

    public boolean getBooleanNT(String name, boolean defaultValue) {
        return _tunerTab.getEntry(name).getBoolean(defaultValue);
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
            _pidcontroller.setOutputRange(min, max);
            _kMinOutput = min; _kMaxOutput = max;
        }
        if((maxV != _kMaxVelocity)) { config_SmartMotionCruiseVelocity(maxV); _kMaxVelocity = maxV; }
        if((minV != _kMinVelocity)) { config_SmartMotionMinVelocity(minV); _kMinVelocity = minV; }
        if((maxA != _kMaxAcceleration)) { config_SmartMotionMaxAcceleration(maxA); _kMaxAcceleration = maxA; }
        if((allE != _kMaxClosedLoopError)) { config_MaxAllowedClosedLoopError(allE); _kMaxClosedLoopError = allE; }


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


    public CANSparkMax getMotor() {
        return _motor;
    }

    public SparkMaxPIDController getPID() {
        return _pidcontroller;
    }
}
