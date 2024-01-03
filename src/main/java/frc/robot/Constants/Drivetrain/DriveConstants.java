package frc.robot.Constants.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldData.FieldConstants;
import frc.utils.AsymmetricalSlewRateLimiter;
//import com.cyberbotics.webots.controller.*;
public final class DriveConstants {
    public static final double kDriveDeadband = 0.05;


    public static final boolean isFieldRelative = true;
    public static final boolean toRateLimit = true;
    public static final double TURBOrate = 1.0;
    public static final double TURBORotScalar = 0.3;
    public static final double BaseDriveRate = 0.2;
    public static final double BaseRotScalar = 0.3;



    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2*kMaxAngularSpeed; // radians per second
    public static final double kMagnitudeSlewRate = 2*kMaxSpeedMetersPerSecond; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 3.0; // percent per second (1 = 100%)
    public static final double drivestickCurve = 2;

    // Asymetrical SlewRateLimiter values;
    public static final AsymmetricalSlewRateLimiter kFancyAccelerationLimiter = new AsymmetricalSlewRateLimiter(

            0.1, 0.3, 5, 0.0, 0.2
    );


    // Chassis configuration
//    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
//    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
//    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
//            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
//            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
//            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
//            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
//    );

    public static double frameWidth = Units.inchesToMeters(25);
    public static double frameLength = Units.inchesToMeters(30);
    //TODO Add actual bumper thickness
    public static double bumperThickness = Units.inchesToMeters(3.5);
    public static double robotCentertoBumper = (frameLength/2) + bumperThickness;
//    public static double RotCenterToWall = ;
//    public static double rotCentertoRobotCenter = (frameLength/2) - (FieldConstants.ArmZerotoBumper-bumperThickness);

    public static double halfRobotLength = (frameLength/2) + bumperThickness;
//    public static double robotCentertoWall = (frameLength/2) + bumperThickness + FieldConstants.BlueAlliance.ScoreAreaDepth;
    public static double scoreApproachBuffer = 0.06;
    public static double AlignSSBuffer = 0.25;

//    public static double scoreDist = robotCentertoWall+scoreApproachBuffer;



    public static double wheelhalfthick = 0.029/2.0;
    public static double wheeledgetoframeedgewidth = 0.104;

    public static double wheeledgetoframeedgelength = 0.1056;

//    public static
    public static double widthoffset = wheeledgetoframeedgewidth-wheelhalfthick;
    public static double lengthoffset = wheeledgetoframeedgelength-wheelhalfthick;

    public static double framewidth = Units.inchesToMeters(15.5);//0.635;
    public static double framelength = Units.inchesToMeters(21.625);
    public static Translation2d flmod = new Translation2d(
            framelength/2-lengthoffset,
            framewidth/2-widthoffset
    );

    public static Translation2d frmod = new Translation2d(
            framelength/2-lengthoffset,
            -framewidth/2+widthoffset
    );

    public static Translation2d rlmod = new Translation2d(
            -framelength/2+lengthoffset,
            framewidth/2-widthoffset
    );

    public static Translation2d rrmod = new Translation2d(
            -framelength/2+lengthoffset,
            -framewidth/2+widthoffset
    );

    public static Translation2d[] modulePositions = new Translation2d[] {
            flmod,
            frmod,
            rlmod,
            rrmod
    };

    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            modulePositions
    );
//    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
//            new Translation2d(
//                    Units.inchesToMeters(11.75-0.35),
//                    Units.inchesToMeters(9.25)
//            ),
//            new Translation2d(
//                    Units.inchesToMeters(11.75-0.35),
//                    Units.inchesToMeters(-9.25)
//            ),
//            new Translation2d(
//                    Units.inchesToMeters(-11.75+0.35),
//                    Units.inchesToMeters(9.25)
//            ),
//            new Translation2d(
//                    Units.inchesToMeters(-11.75+0.35),
//                    Units.inchesToMeters(-9.25)
//            )
//    );

    // Angular offsets of the modules relative to the chassis in radians

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int backRightSteerCanId = 15;
    public static final int backRightDriveCanId = 14;
    public static final int backLeftDriveCanId = 16;
    public static final int backLeftSteerCanId = 17;
    public static final int frontRightDriveCanId = 12;
    public static final int frontRightSteerCanId = 13;
    public static final int frontLeftDriveCanId = 10;
    public static final int frontLeftSteerCanId = 11;

    public static final boolean kGyroReversed = false;

    public static class PIDConstants {
        public static double xtracP = 2.0;
        public static double xtracI = 0.0;
        public static double xtracD = 0.0;
        public static double xtracTolerance = 0.02;

        public static double ytracP = 2.0;
        public static double ytracI = 0.0;
        public static double ytracD = 0.0;
        public static double ytracTolerance = 0.02;

        public static double rtracP = 4.0;
        public static double rtracI = 0.0;
        public static double rtracD = 0.0;
        public static double rtracTolerance = Units.degreesToRadians(1.0);

        public static double magtracP = 2.0;
        public static double magtracI = 0.0;
        public static double magtracD = 0.0;
        public static double magtracTolerance = 0.02;



        public static TrapezoidProfile.Constraints baseXYConstraints = new TrapezoidProfile.Constraints(
                kMaxSpeedMetersPerSecond/3,
                kMaxSpeedMetersPerSecond/2
        );
        public static TrapezoidProfile.Constraints baseRotConstraints = new TrapezoidProfile.Constraints(
                Units.degreesToRadians(180),
                Units.degreesToRadians(180)
        );

        public static ProfiledPIDController infinityPID = new ProfiledPIDController(
                Double.MAX_VALUE,
                0.0,
                0.0,
                baseXYConstraints
        );

        public static double nextStepDistance = 0.1;


    }

}
