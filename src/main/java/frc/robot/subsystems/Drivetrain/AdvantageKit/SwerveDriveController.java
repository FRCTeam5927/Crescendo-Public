package frc.robot.subsystems.Drivetrain.AdvantageKit;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.IOManager;
import frc.robot.SubManager;
import frc.robot.libs.MathLib;
import frc.robot.libs.interfaces.DriveController;
import frc.robot.subsystems.Sensors.PigeonSubsystem;
import frc.utils.SwerveUtils;
import org.littletonrobotics.junction.Logger;

import java.util.stream.StreamSupport;

import static java.lang.Math.abs;

public class SwerveDriveController extends SubsystemBase implements DriveController {

    private boolean toRateLimit = DriveConstants.toRateLimit;
    private boolean isFieldRelative = DriveConstants.isFieldRelative;

    SwerveModule frontLeft = new SwerveModule(0);
    SwerveModule frontRight = new SwerveModule(1);
    SwerveModule backLeft = new SwerveModule(2);
    SwerveModule backRight = new SwerveModule(3);
    SwerveModule[] dtMods = new SwerveModule[] {
            frontLeft, frontRight, backLeft, backRight
    };

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            DriveConstants.flmod,
            DriveConstants.frmod,
            DriveConstants.rlmod,
            DriveConstants.rrmod
    );

    PigeonSubsystem pigeonSubsystem;
    SwerveDriveOdometry driveOdometry;

    private double oldPoseTimestamp = 0.0;
    private double oldoldPoseTimestamp = 0.0;

    private double _currentTranslationMag = 0.0;
    private double _currentTranslationDir = 0.0;
    private double m_prevTime = 0.0;

    SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kDirectionSlewRate);


    String logKey = "SwerveDriveController/";

    public SwerveDriveController() {

        pigeonSubsystem = SubManager.get_pigeon();

        driveOdometry = new SwerveDriveOdometry(
                kinematics,
                new Rotation2d(pigeonSubsystem.getFusedHeading()),
                getModulePositions()
        );
        resetHeading();
    }

    @Override
    public void periodic() {
        driveOdometry.update(
                new Rotation2d(pigeonSubsystem.getFusedHeading()),
                getModulePositions()
        );
        oldoldPoseTimestamp = oldPoseTimestamp;
        oldPoseTimestamp = Timer.getFPGATimestamp();

        Logger.recordOutput(logKey+"DSR_Pose", getPoseDSR());
        Logger.recordOutput(logKey+"FABS_Pose", getPoseFABS());
        Logger.recordOutput(logKey+"ModulePositions", getModulePositions());
        Logger.recordOutput(logKey+"ModuleStates", getModuleStates());



        for(int i=0; i< dtMods.length; i++) {
            SmartDashboard.putString("smodp"+i, dtMods[i].getModulePosition().toString());


        }
    }


    @Override
    public void drive(double xSpeedMeters, double ySpeedMeters, double angleSpeedRadians, double throttle) {
        Logger.recordOutput(logKey+"RAW_xSpeedMeters", xSpeedMeters);
        Logger.recordOutput(logKey+"RAW_ySpeedMeters", ySpeedMeters);
        Logger.recordOutput(logKey+"RAW_thetaSpeedRadians", angleSpeedRadians);
        Logger.recordOutput(logKey+"RAW_throttle", xSpeedMeters);



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
        Logger.recordOutput(logKey+"RAW_DriveSpeedCommanded", speedMeters);
        Logger.recordOutput(logKey+"RAW_AngleSpeedCommanded", angleSpeedRadians);
        Logger.recordOutput(logKey+"RAW_HeadingRadians", headingRadians);




        double xSpeedCommanded;
        double ySpeedCommanded;
        double _currentRotation;
        if (toRateLimit) {
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

        Logger.recordOutput(logKey+"currentTranslationMag", _currentTranslationMag);
        Logger.recordOutput(logKey+"currentTranslationDir", _currentTranslationDir);

        setChassisSpeeds(rawChassisSpeeds);
    }


    @Override
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for(int i=0; i<moduleStates.length; i++) {
            dtMods[i].setDesiredState(moduleStates[i]);
            SmartDashboard.putString("smodc"+i, moduleStates[i].toString());
//            SmartDashboard.putString("smodp"+i, dtMods[i].getModulePosition().toString());


        }

    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        if(isFieldRelative){
            setDSRSpeeds(speeds);
        } else {
            setRRSpeeds(speeds);
        }

    }


    @Override
    public void setRRSpeeds(ChassisSpeeds RRSpeeds) {
        Logger.recordOutput(logKey+"RRS_Commanded", RRSpeeds);
        setModuleStates(kinematics.toSwerveModuleStates(RRSpeeds));
    }

    @Override
    public void setDSRSpeeds(ChassisSpeeds DSRSpeeds) {
        Logger.recordOutput(logKey+"DSR_Commanded", DSRSpeeds);
        setRRSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        DSRSpeeds, new Rotation2d(pigeonSubsystem.getFusedHeading())
                )
        );

    }

    @Override
    public void setFABSSpeeds(ChassisSpeeds FABSSpeeds) {
        Logger.recordOutput(logKey+"FABS_Commanded", FABSSpeeds);
        setDSRSpeeds(MathLib.flipXYIfRed(FABSSpeeds));
    }

    @Override
    public void setX() {
        for(int i=0; i < dtMods.length; i++) {
            dtMods[i].setModuleAngle(DriveConstants.modulePositions[i].getAngle().getDegrees());
        }
    }

    @Override
    public void resetHeading() {
        resetHeading(0);
    }

    @Override
    public void resetHeading(double angle) {
        pigeonSubsystem.setYaw(angle);
        pigeonSubsystem.setFusedYaw(angle);
    }

    @Override
    public void resetOdometry(Pose2d resetPose) {
        driveOdometry.resetPosition(
                new Rotation2d(pigeonSubsystem.getFusedHeading()),
                getModulePositions(),
                resetPose
        );
    }

    @Override
    public void setOdometryAngle(Rotation2d angle) {
        driveOdometry.resetPosition(
                angle,
                getModulePositions(),
                getPoseDSR()
        );
    }

    @Override
    public void config_rateLimit(boolean enabled) {
        toRateLimit = enabled;
    }

    @Override
    public void config_fieldRelative(boolean enabled) {
        isFieldRelative = enabled;
    }

    @Override
    public void tuneSwerveModuleAngle() {
        frontLeft.tuneAngle();
    }

    @Override
    public void tuneSwerveModuleDrive() {
        frontLeft.tuneDrive();
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[dtMods.length];
        for(int i =0; i < dtMods.length; i++) {
            moduleStates[i] = dtMods[i].getModuleState();
        }
        return moduleStates;
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] moduleStates = new SwerveModulePosition[dtMods.length];
        for(int i =0; i < dtMods.length; i++) {
            moduleStates[i] = dtMods[i].getModulePosition();
        }
        return moduleStates;
    }

    @Override
    public Pose2d getPose() {
        return getPoseDSR();
    }

    @Override
    public double getOldPoseTimestamp() {
        return oldPoseTimestamp;
    }

    @Override
    public double getOldOldPoseTimestamp() {
        return oldoldPoseTimestamp;
    }

    @Override
    public Pose2d getPoseDSR() {
        return driveOdometry.getPoseMeters();
    }

    @Override
    public Pose2d getPoseFABS() {
        return MathLib.translateDSRtoFABS(getPoseDSR());
    }

    @Override
    public ChassisSpeeds getRRSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public ChassisSpeeds getDSRSpeeds() {
        return MathLib.rotateSpeeds(pigeonSubsystem.getFusedHeading(), getRRSpeeds());
    }

    @Override
    public ChassisSpeeds getFABSSpeeds() {
        ChassisSpeeds DSRSpeeds = getDSRSpeeds();
        return MathLib.flipXYIfRed(DSRSpeeds);
    }


    @Override
    public boolean get_fieldRelative() {
        return isFieldRelative;
    }

    @Override
    public boolean get_rateLimit() {
        return toRateLimit;
    }
}
