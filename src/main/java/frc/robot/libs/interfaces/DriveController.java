package frc.robot.libs.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DriveController {
    void drive(double xSpeedMeters, double ySpeedMeters, double angleSpeedRadians, double throttle);
    void vectorDrive(double headingRadians, double speedMeters, double angleSpeedRadians);

    void setModuleStates(SwerveModuleState[] moduleStates);

    void setChassisSpeeds(ChassisSpeeds speeds);
    /**
     *
     * @param RRSpeeds robot-relative chassis speeds,
     * where +x is forwards relative to the robot's frame
     */
    void setRRSpeeds(ChassisSpeeds RRSpeeds);

    /**
     *
     * @param DSRSpeeds
     * Takes in Chassis speeds relative to the current Alliance's Driver station.
     */
    void setDSRSpeeds(ChassisSpeeds DSRSpeeds);
    /**
     * @param FABSSpeeds
     * Takes in Chassis speeds that will move the robot in a way
     *  that is conformant to the field coordinate system.
     *  Useful for autonomous movement.
     */
    void setFABSSpeeds(ChassisSpeeds FABSSpeeds);
    /**
     * @return Chassis speeds of the robot in the context of the field coordinate system.
     * Useful for autonomous movement checking.
     */
    void setX();
    void resetHeading();
    void resetHeading(double angle);
    void resetOdometry(Pose2d resetPose);
    void setOdometryAngle(Rotation2d angle);
    void config_rateLimit(boolean enabled);
    void config_fieldRelative(boolean enabled);
    void tuneSwerveModuleAngle();
    void tuneSwerveModuleDrive();




    SwerveModuleState[] getModuleStates();
    SwerveModulePosition[] getModulePositions();
    Pose2d getPose();
    double getOldPoseTimestamp();
    double getOldOldPoseTimestamp();

    Pose2d getPoseDSR();
    Pose2d getPoseFABS();


    /**
     *
     * @return ChassisSpeeds,
     * where +x is forwards relative to the robot's frame
     *
     */
    ChassisSpeeds getRRSpeeds();

    /**
     *
     * @return Chassis speeds relative to the current Alliance's Driver station.
     */
    ChassisSpeeds getDSRSpeeds();

    ChassisSpeeds getFABSSpeeds();

    boolean get_fieldRelative();
    boolean get_rateLimit();

}
