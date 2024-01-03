package frc.robot.libs.BaseMechanism;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.SubManager;

import java.util.function.DoubleConsumer;


public class TalonFXMechanism implements BaseMechanism, Subsystem {
    private static double POSITION_ERROR_TOLERANCE = 1;
    TalonFX _motor;
    TalonFXConfigurator _configurator;
    TalonFXConfiguration masterConfig;


    DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0).withUpdateFreqHz(10);
    VoltageOut voltageControl = new VoltageOut(0.0).withUpdateFreqHz(10);
    MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0.0).withUpdateFreqHz(10);
    PositionVoltage racePIDControl = new PositionVoltage(0.0).withUpdateFreqHz(10);

    VelocityVoltage velocitycontrol = new VelocityVoltage(0.0).withUpdateFreqHz(10);

    DoubleConsumer _positionSetter = this::setSmartPosition;
    double _maxPosition = Double.MAX_VALUE;
    double _minPosition = Double.MIN_VALUE;
    DoubleConsumer _velocitySetter = this::setBasicVelocity;
//    TalonFXSensorCollection _sensors;
    CANcoder _cancoder;
    int remoteCanID;
    protected NetworkTable _tunerTab;

    SendableChooser<ControlModes.PositionMode> positionModeSelector = new SendableChooser<>();
    ControlModes.PositionMode _positionMode = ControlModes.PositionMode.SmartMotion;
    double debugPos = 0.0;
    boolean debugToTrack = false;
    boolean vel_or_pos = false;
    double _kP;
    double _kI;
    double _kD;
    double _kV;
    double _kS;
    double _kG;
    double _kA;
    boolean _HasPIDTunerRun = false;
    double _kMinVelocity;
    double _kMaxVelocity;
    double _kMaxAcceleration;
    double _kSmartMotionCurveStrength;
    double _kMaxClosedLoopError;
    double _maxPositionSetpoint = Double.MAX_VALUE;
    double _minPositionSetpoint = Double.MIN_VALUE;
    double POSITION_CONVERSION_FACTOR = 1;
    double VELOCITY_CONVERSION_FACTOR = 1;
    double ABSOLUTE_POSITION_CONVERSION_FACTOR = 1;
    double ABSOLUTE_VELOCITY_CONVERSION_FACTOR = 1;
    double RELATIVE_POSITION_CONVERSION_FACTOR = 1;
    double RELATIVE_VELOCITY_CONVERSION_FACTOR = 1;
    boolean ABSOLUTE_MODE = false;

    Timer holdChecker = new Timer();
    boolean firstLoop = true;
    double holdPosition;

    String motorID;


    public TalonFXMechanism(int canID) {
        _motor = new TalonFX(canID);
//        _sensors
        _configurator = _motor.getConfigurator();
        masterConfig = new TalonFXConfiguration();
        _motor.setSafetyEnabled(false);
        _motor.setExpiration(0.1);
        motorID = "TalonFXMechanism_ID_"+_motor.getDeviceID();
        _tunerTab = _netinst.getTable("TalonFXMechanism_ID_"+_motor.getDeviceID());

    }


    @Override
    public int getMotorID() {
        return _motor.getDeviceID();
    }

    @Override
    public void setRawPercentOutput(double pct) {

            _motor.set(pct);

    }

    @Override
    public void setThrottle(double pct) {
        _motor.setControl(
            dutyCycleControl.
            withOutput(pct + (renderArbFF()/getSupplyVoltage()))
        );
    } 

    @Override
    public void setVoltage(double volts) {
        _motor.setControl(
            voltageControl.withOutput(volts+renderArbFF())
        );
    }

    @Override
    public void setVelocity(double speed) {
        _velocitySetter.accept(speed);
    }

    double simPosition = 0.0;
    double simVelocity = 0.0;
    boolean isSimulation = Robot.isSimulation();
    @Override
    public void setBasicVelocity(double speed) {
        if(isSimulation) {
            simPosition+=(speed*0.02);
            simVelocity = speed;

        } else {
            _motor.setControl(
                    velocitycontrol.
                            withVelocity(speed/VELOCITY_CONVERSION_FACTOR).
                            withFeedForward(renderArbFF())
            );
        }

    }

    @Override
    public double getVelocity() {
        if(isSimulation) {
            return simVelocity;
        } else {
            return _motor.getVelocity().getValue()*VELOCITY_CONVERSION_FACTOR;
        }
    }

    public double getAbsoluteVelocity() {
        return _cancoder.getVelocity().getValue()*ABSOLUTE_VELOCITY_CONVERSION_FACTOR;
    }

    @Override
    public void setPosition(double position) {
        _positionSetter.accept(position);
    }

    @Override
    public void setRacePIDPosition(double position) {
       position = constrainSetpoint(position);
       position/=POSITION_CONVERSION_FACTOR;
        if(isSimulation) {
            simPosition = position;
        }
        _motor.setControl(
            racePIDControl.
                withPosition(position).
                withFeedForward(renderArbFF())
        );
    }

    @Override
    public void setSmartPosition(double position) {
       position = constrainSetpoint(position);
       position/=POSITION_CONVERSION_FACTOR;
        if(isSimulation) {
            simPosition = position;
        }
        _motor.setControl(
            motionMagicControl
                .withPosition(position/POSITION_CONVERSION_FACTOR).
                withFeedForward(renderArbFF())
        );
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
        if(isSimulation) {
            return simPosition;
        } else {
            if(ABSOLUTE_MODE) {
                return _cancoder.getAbsolutePosition().getValueAsDouble()*ABSOLUTE_POSITION_CONVERSION_FACTOR;
            } else {
                return _motor.getPosition().getValue()*RELATIVE_POSITION_CONVERSION_FACTOR;

            }
        }
    }

    @Override
    public double getSupplyVoltage() {
        return _motor.getSupplyVoltage().getValueAsDouble();
    }

    @Override
    public double getAppliedCurrent() {
        return _motor.getStatorCurrent().getValue();
    }

    @Override
    public double getTemperature() {
        return _motor.getDeviceTemp().getValue();
    }

    public double getAbsolutePosition() {
        return _cancoder.getAbsolutePosition().getValue()*ABSOLUTE_POSITION_CONVERSION_FACTOR;
    }

    @Override
    public double renderArbFF() {
        double arbFF = 0.0;
        arbFF += _kG * Math.sin(Units.degreesToRadians(getPosition()));
        return arbFF;
    }

    @Override
    public double getIntegratedSensorPosition() {
        return _motor.getPosition().getValue();
    }

    @Override
    public double getAbsoluteSensorPosition() {
        return _cancoder.getAbsolutePosition().getValue()*ABSOLUTE_POSITION_CONVERSION_FACTOR;
    }

    @Override
    public double getSelectedSensorPosition() {
        return _motor.getPosition().getValue()*POSITION_CONVERSION_FACTOR;
    }

    @Override
    public double getSelectedSensorVelocity() {
        return _motor.getVelocity().getValue()*VELOCITY_CONVERSION_FACTOR;
    }

    @Override
    public double constrainSetpoint(double sp) {
        return MathUtil.clamp(sp, _minPosition, _maxPosition);
    }

    @Override
    public boolean isAtPoint(double point) {
        return Math.abs(point-getPosition())<POSITION_ERROR_TOLERANCE;
    }

    @Override
    public boolean isAtPoint(double point, double tolerance) {
        return Math.abs(point-getPosition())<tolerance;
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
        switch (idleMode){
            case kCoast:
                masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                break;
            case kBrake:
                masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                break;
            default:
                System.out.println("INVALID NEUTRAL TYPE");
        }
        _configurator.apply(masterConfig);


    }

    @Override
    public void config_SmartMotionConstraints(TrapezoidProfile.Constraints constraints) {
        config_SmartMotionCruiseVelocity(constraints.maxVelocity);
        config_SmartMotionMaxAcceleration(constraints.maxAcceleration);
    }

    @Override
    public void config_SmartMotionCruiseVelocity(double vel) {
        _kMaxVelocity = vel;
        masterConfig.MotionMagic.MotionMagicCruiseVelocity = vel/VELOCITY_CONVERSION_FACTOR;
        _configurator.apply(masterConfig);
    }


    @Override
    public void config_SmartMotionMaxAcceleration(double vel_squared) {
        _kMaxAcceleration = vel_squared;
        masterConfig.MotionMagic.MotionMagicAcceleration = vel_squared/VELOCITY_CONVERSION_FACTOR;
        _configurator.apply(masterConfig);
    }

    public void config_SmartMotionCurveStrength(double power) {
        masterConfig.MotionMagic.MotionMagicJerk = power/VELOCITY_CONVERSION_FACTOR;
        _configurator.apply(masterConfig);
    }


    @Override
    public void config_kP(double kP) {
        _kP = kP;
        masterConfig.Slot0.kP = kP;
        _configurator.apply(masterConfig);
    }

    @Override
    public void config_kI(double kI) {
        _kI = kI;
        masterConfig.Slot0.kI = kI;
        _configurator.apply(masterConfig);

    }

    @Override
    public void config_kIz(double kIz) {
//        _motor.config_IntegralZone(0, kIz);
    }

    @Override
    public void config_kD(double kD) {
        _kD = kD;
        masterConfig.Slot0.kD = kD;
        _configurator.apply(masterConfig);

    }

    @Override
    public void config_kV(double kV) {
        _kV = kV;
        masterConfig.Slot0.kV = kV;
        _configurator.apply(masterConfig);
    }

    @Override
    public void config_kS(double kS) {
        _kS = kS;
        masterConfig.Slot0.kS = kS;
        _configurator.apply(masterConfig);
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
//        _motor.configClosedLoopPeakOutput(0, upperbound/POSITION_CONVERSION_FACTOR);
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

            default:
                System.out.println("INVALID PositionMode");

        }

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
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimit = maxAmps;
        masterConfig.CurrentLimits.SupplyCurrentThreshold = maxAmps*1.1;
        masterConfig.CurrentLimits.SupplyTimeThreshold = 0.2;
        _configurator.apply(masterConfig);
    }

    @Override
    public void config_ClosedLoopRampRate(double seconds) {
        masterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = seconds;
        masterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = seconds;
        masterConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod =  seconds;

        _configurator.apply(masterConfig);
    }

    @Override
    public void config_OpenLoopRampRate(double seconds) {
        masterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = seconds;
        masterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = seconds;
        masterConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = seconds;

        _configurator.apply(masterConfig);
    }

    @Override
    public void setIntegratedSensorPosition(double position) {
        _motor.setPosition(position/RELATIVE_POSITION_CONVERSION_FACTOR);
    }

    public void setRemoteSensorCanID(int ID) {
        remoteCanID = ID;
        _cancoder = new CANcoder(ID);
    }

    @Override
    public void config_AbsoluteMode(boolean absoluteMode) {
        ABSOLUTE_MODE = absoluteMode;
        if(absoluteMode) {
//            _cancoder = new CANcoder(remoteCanID);
            masterConfig.Feedback.FeedbackRemoteSensorID = remoteCanID;
            masterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
//            _feedbackConfigs.
            _configurator.apply(masterConfig);
            POSITION_CONVERSION_FACTOR = ABSOLUTE_POSITION_CONVERSION_FACTOR;
            VELOCITY_CONVERSION_FACTOR = ABSOLUTE_VELOCITY_CONVERSION_FACTOR;
        } else {
            masterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            _configurator.apply(masterConfig);
            POSITION_CONVERSION_FACTOR = RELATIVE_POSITION_CONVERSION_FACTOR;
            VELOCITY_CONVERSION_FACTOR = RELATIVE_VELOCITY_CONVERSION_FACTOR;
        }

    }

    /**
     * saveConfigs is a no-op in TalonFXMechanism because configs automatically save
     */
    @Override
    public void saveConfigs() {
    }

    @Override
    public void restoreFactoryDefault() {
        _configurator.apply(new TalonFXConfiguration());
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

            _tunerTab = _netinst.getTable("TalonFXMechanism_ID_"+_motor.getDeviceID());

            putNumberNT("SetPosition", 0.0);
            putBooleanNT("ToTrack", false);
            putBooleanNT("VEl_OR_POS", false);
            positionModeSelector.setDefaultOption("SmartMotion", ControlModes.PositionMode.SmartMotion);
            positionModeSelector.addOption("RacePID", ControlModes.PositionMode.RacePID);


            ShuffleboardTab myTab = Shuffleboard.getTab("TalonFXMechanism_ID_"+_motor.getDeviceID());
            myTab.add("PositionModeSelector", positionModeSelector);

//            myTab.buildInto(_tunerTab);



            putNumberNT("PID_P Gain", _kP);
            putNumberNT("PID_I Gain", _kI);
            putNumberNT("PID_D Gain", _kD);
//            SmartDashboard.putNumber("I Zone", _kIz);
            putNumberNT("Feed Forward (Ordinary)", _kV);
            putNumberNT("Feed Forward (Static Friction Compensation)", _kS);
            putNumberNT("Feed Forward (Gravity Compensation)", _kG);
            putNumberNT("Feed Forward (Acceleration Compensation)", _kA);


//            SmartDashboard.getNumber("Max Output", 0);
//            SmartDashboard.getNumber("Min Output", 0);
            putNumberNT("Max Velocity", 0);
//            SmartDashboard.getNumber("Min Velocity", 0);
            putNumberNT("Max Acceleration", 0);
            putNumberNT("Curve Smoothing", 0);
//            SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

            _HasPIDTunerRun = true;
        }

        double debugpos = getNumberNT("SetPosition", 0.0);
        boolean toTrack = getBooleanNT("ToTrack", false);
        boolean velorpos = getBooleanNT("VEl_OR_POS", false);
        ControlModes.PositionMode cmod = positionModeSelector.getSelected();

        double p = getNumberNT("PID_P Gain", 0);
        double i = getNumberNT("PID_I Gain", 0);
        double d = getNumberNT("PID_D Gain", 0);
//        double iz = SmartDashboard.getNumber("I Zone", 0);
        double kV = getNumberNT("Feed Forward (Ordinary)", 0);
        double kS = getNumberNT("Feed Forward (Static Friction Compensation)", 0);
        double kG = getNumberNT("Feed Forward (Gravity Compensation)", 0);
        double kA = getNumberNT("Feed Forward (Acceleration Compensation)", 0);


//        double max = SmartDashboard.getNumber("Max Output", 0);
//        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = getNumberNT("Max Velocity", 0);
//        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = getNumberNT("Max Acceleration", 0);
        double smoother = getNumberNT("Curve Smoothing", 0);

//        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
//        new MotorOutputConfigs().DutyCycleNeutralDeadband;
//        new MotionMagicConfigs().

        if(debugpos != debugPos) { debugPos = debugpos; }
        if(toTrack != debugToTrack) { debugToTrack = toTrack; }
        if(velorpos != vel_or_pos) { vel_or_pos = velorpos; }



        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != _kP)) { config_kP(p); }
        if((i != _kI)) { config_kI(i); }
        if((d != _kD)) { config_kD(d); }
//        if((iz != _kIz)) { config_kIz(iz); }
        if((kV != _kV)) { config_kV(kV); }
        if((kS != _kS)) { config_kS(kS); }
        if((kS != _kG)) { config_kG(kG); }
        if((kS != _kA)) { config_kA(kA); }

        //currently not implemented but
//        if((max != _kMaxOutput) || (min != _kMinOutput)) {
//            _pidcontroller.setOutputRange(min, max);
//            _kMinOutput = min; _kMaxOutput = max;
//        }
        if((maxV != _kMaxVelocity)) { config_SmartMotionCruiseVelocity(maxV); _kMaxVelocity = maxV; }
//        if((minV != _kMinVelocity)) { _pidcontroller.setSmartMotionMinOutputVelocity(minV,0); _kMinVelocity = minV; }
        if((maxA != _kMaxAcceleration)) { config_SmartMotionMaxAcceleration(maxA); _kMaxAcceleration = maxA; }
        if((smoother != _kSmartMotionCurveStrength)) { config_SmartMotionCurveStrength(smoother); _kSmartMotionCurveStrength = maxA; }

//        if((allE != _kMaxClosedLoopError)) { _pidcontroller.setSmartMotionAllowedClosedLoopError(allE,0); _kMaxClosedLoopError = allE; }


        if(cmod != _positionMode) {
            _positionMode = cmod;
            config_PositionMode(_positionMode);
        }
        if(debugToTrack) {
            if(vel_or_pos) {
                setBasicVelocity(debugPos);
            } else {
                setPosition(debugPos);
            }

        } else {
            setRawPercentOutput(0.0);
        }


        if(ABSOLUTE_MODE) {
            putNumberNT("Error",debugPos- getAbsolutePosition());
            putNumberNT("VelError", debugPos-getAbsoluteVelocity());
        } else {
            putNumberNT("Error",debugPos-getPosition());
            putNumberNT("VelError", debugPos-getVelocity());
        }


    }


    public void config_rotorToSensor(double factor) {
        masterConfig.Feedback.RotorToSensorRatio = factor;
        _configurator.apply(masterConfig);
    }

    public void config_sensorToMechanism(double factor) {
        masterConfig.Feedback.SensorToMechanismRatio = factor;
        _configurator.apply(masterConfig);
    }

    public TalonFX getMotor() {
        return _motor;
    }
}
