package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.Robot;
import frc.robot.libs.DiagnosticTable;
import frc.robot.libs.MathLib;
import frc.robot.libs.SimOdometer;
import frc.robot.libs.interfaces.DriveController;
import frc.robot.subsystems.Sensors.IMUSubsystem;
import frc.utils.SwerveUtils;

import static java.lang.Math.abs;

public class DriveSubsystem extends SubsystemBase implements DriveController {

    boolean _fieldRelative = DriveConstants.isFieldRelative;
    boolean _toRateLimit = DriveConstants.toRateLimit;


    // Create SwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.frontLeftDriveCanId,
            DriveConstants.frontLeftSteerCanId,
            DriveConstants.kFrontLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.frontRightDriveCanId,
            DriveConstants.frontRightSteerCanId,
            DriveConstants.kFrontRightChassisAngularOffset
    );

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.backLeftDriveCanId,
            DriveConstants.backLeftSteerCanId,
            DriveConstants.kBackLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.backRightDriveCanId,
            DriveConstants.backRightSteerCanId,
            DriveConstants.kBackRightChassisAngularOffset
    );

    MAXSwerveModule[] swerveModules = new MAXSwerveModule[]{
            m_frontLeft, m_frontRight, m_rearLeft, m_rearRight
    };
    // The gyro sensor
    private final IMUSubsystem _pigeon = IMUSubsystem.getInstance();
    private double _currentTranslationDir = 0.0;
    private double _currentTranslationMag = 0.0;

    private final SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    public SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry _odometry = new SwerveDriveOdometry(
            m_kinematics,
            Rotation2d.fromDegrees(_pigeon.getYaw()),
            getModulePositions()
    );

    DiagnosticTable dtab = new DiagnosticTable("Drivetrain");
    double lastodotimestamp = Timer.getFPGATimestamp();
    double lastoldodotimestamp = Timer.getFPGATimestamp();
    Field2d dsrfield = new Field2d();

    public MAXSwerveModule[] getAllSwerves() {
        return swerveModules;
    }

    public DriveSubsystem() {
        SmartDashboard.putData("ODOFIELD", dsrfield);
        resetHeading();
//        config_rateLimit(false);
    }
    @Override
    public void drive(double xSpeedMeters, double ySpeedMeters, double angleSpeedRadians, double throttle) {
        SmartDashboard.putNumber("xs", xSpeedMeters);
        double inputTranslationDir = Math.atan2(xSpeedMeters, ySpeedMeters);
        double _drivestickCurve = DriveConstants.drivestickCurve;
        double inputTranslationMag = Math.pow(Math.sqrt(Math.pow(xSpeedMeters, 2) + Math.pow(ySpeedMeters, 2)), _drivestickCurve)
                *(DriveConstants.BaseDriveRate +((1.0-DriveConstants.BaseDriveRate)*throttle));

        SmartDashboard.putNumber("imag",inputTranslationMag);
        vectorDrive(inputTranslationDir, inputTranslationMag, angleSpeedRadians);

    }

    @Override
    public void vectorDrive(double headingRadians, double speedMeters, double angleSpeedRadians) {
        double xSpeedCommanded;
        double ySpeedCommanded;
        double _currentRotation;
        if (_toRateLimit) {
            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (_currentTranslationMag != 0.0) {
                directionSlewRate = abs(DriveConstants.kDirectionSlewRate / _currentTranslationMag);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;

            double angleDif = SwerveUtils.AngleDifference(headingRadians, _currentTranslationDir);

            if (angleDif < 0.45*Math.PI) {
                _currentTranslationDir = SwerveUtils.StepTowardsCircular(_currentTranslationDir, headingRadians, directionSlewRate * elapsedTime);
                _currentTranslationMag = m_magLimiter.calculate(speedMeters);
            }
            else if (angleDif > 0.85*Math.PI) {
                if (_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    _currentTranslationMag = m_magLimiter.calculate(0.0);
                }
                else {
                    _currentTranslationDir = SwerveUtils.WrapAngle(_currentTranslationDir + Math.PI);
                    _currentTranslationMag = m_magLimiter.calculate(speedMeters);
                }
            }
            else {
                _currentTranslationDir = SwerveUtils.StepTowardsCircular(_currentTranslationDir, speedMeters, directionSlewRate * elapsedTime);
                _currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;



            xSpeedCommanded = _currentTranslationMag * Math.cos(_currentTranslationDir);
            ySpeedCommanded = _currentTranslationMag * Math.sin(_currentTranslationDir);
            _currentRotation = m_rotLimiter.calculate(angleSpeedRadians);
        } else {
            xSpeedCommanded = speedMeters*Math.cos(headingRadians);
            ySpeedCommanded = speedMeters*Math.sin(headingRadians);
            _currentRotation = angleSpeedRadians;
        }


        ChassisSpeeds rawChassisSpeeds = new ChassisSpeeds(
                xSpeedCommanded*DriveConstants.kMaxSpeedMetersPerSecond,
                ySpeedCommanded*DriveConstants.kMaxSpeedMetersPerSecond,
                _currentRotation*DriveConstants.kRotationalSlewRate
        );
        SmartDashboard.putString("rs", rawChassisSpeeds.toString());
        SmartDashboard.putNumber("mags", _currentTranslationMag);
        setChassisSpeeds(rawChassisSpeeds);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for(int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(moduleStates[i]);
        }
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        if(_fieldRelative){
            setDSRSpeeds(speeds);
        } else {
            setRRSpeeds(speeds);
        }

    }


    @Override
    public void setRRSpeeds(ChassisSpeeds RRSpeeds) {
        setModuleStates(m_kinematics.toSwerveModuleStates(RRSpeeds));
    }

    SimOdometer simOdo = new SimOdometer();
    @Override
    public void setDSRSpeeds(ChassisSpeeds DSRSpeeds) {
        if(Robot.isSimulation()) {
            simOdo.update(DSRSpeeds);
        }
        setRRSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(DSRSpeeds, new Rotation2d()));//(_pigeon.getDSrelfusedYaw())));
    }

    @Override
    public void setFABSSpeeds(ChassisSpeeds FABSSpeeds) {
        setDSRSpeeds(MathLib.flipXYIfRed(FABSSpeeds));
    }

    @Override
    public void config_rateLimit(boolean enabled) {
        _toRateLimit = enabled;
    }

    @Override
    public void config_fieldRelative(boolean enabled) {
        _fieldRelative = enabled;
    }

    @Override
    public void tuneSwerveModuleAngle() {
//        m_frontLeft.get_angleMechanism().runPIDtuner();
    }

    @Override
    public void tuneSwerveModuleDrive() {
//        m_frontLeft.get_driveMechanism().runPIDtuner();
    }


    @Override
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for(int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];
        for(int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getPosition();
        }
        return states;
    }

    @Override
    public Pose2d getPose() {
        return getPoseDSR();
    }

    @Override
    public double getOldPoseTimestamp() {
        return lastodotimestamp;
    }

    @Override
    public double getOldOldPoseTimestamp() {
        return lastoldodotimestamp;
    }

    @Override
    public Pose2d getPoseDSR() {
        if(Robot.isReal()) {
            return _odometry.getPoseMeters();
        } else {
            return simOdo.getPose();
        }
    }

    @Override
    public Pose2d getPoseFABS() {
        return MathLib.translateDSRtoFABS(getPoseDSR());
    }

    @Override
    public ChassisSpeeds getRRSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public ChassisSpeeds getDSRSpeeds() {
        return MathLib.rotateSpeeds(_pigeon.getDSrelfusedYaw(), getRRSpeeds());
    }

    @Override
    public ChassisSpeeds getFABSSpeeds() {
        ChassisSpeeds DSRSpeeds = getDSRSpeeds();
        return MathLib.flipXYIfRed(DSRSpeeds);
    }

    @Override
    public boolean get_fieldRelative() {
        return _fieldRelative;
    }

    @Override
    public boolean get_rateLimit() {
        return _toRateLimit;
    }


    @Override
    public void setX() {
//        for(SwerveModule smod : swerveModules) {
//            smod.pointToCenter();
//        }
    }

    @Override
    public void resetHeading() {
        resetHeading(0);
    }

    @Override
    public void resetHeading(double angle) {
        _pigeon.updateVisionAngle(angle);
    }

    @Override
    public void resetOdometry(Pose2d resetPose) {
        if(Robot.isReal()) {
            _odometry.resetPosition(
                    new Rotation2d(_pigeon.getDSrelfusedYaw()),
                    getModulePositions(),
                    resetPose
            );
        } else {
            simOdo.resetPose(resetPose);
        }

    }

    @Override
    public void setOdometryAngle(Rotation2d angle) {
        resetOdometry(
                new Pose2d(
                        getPoseDSR().getTranslation(),
                        angle
                )
        );
    }

    @Override
    public void periodic() {
        _odometry.update(
                new Rotation2d(_pigeon.getDSrelfusedYaw()),
                getModulePositions()
        );

        lastoldodotimestamp = lastodotimestamp;
        lastodotimestamp = Timer.getFPGATimestamp();
        dsrfield.setRobotPose(getPoseDSR());

        if(DebugConstants.debugDT) {
            dtab.putString("RRSpeeds", getRRSpeeds().toString());
            dtab.putString("DSRPose", getDSRSpeeds().toString());
        }
    }
}
