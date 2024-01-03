package frc.robot.Constants.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldData.FieldConstants;
import frc.robot.libs.MathLib;

import java.util.ArrayList;
import java.util.List;

public class AutoConstants {

    public static List<Pose2d> transelatePathToRed(List<Pose2d> PoseList) {
        List<Pose2d> exportList = new ArrayList<>();
        for (Pose2d iterpos : PoseList) {
            exportList.add(MathLib.translateForRed(iterpos));
        }
        return exportList;
    }

    public class AutoPieces {
//        public static double scoreAreaDepth = FieldConstants.BlueAlliance.ScoreAreaDepth;
//        public static double autoGPX = scoreAreaDepth+Units.inchesToMeters(18*12+8);
        public static double autoGP_Yoffset = Units.inchesToMeters(36.75);
        public static double autoGPSpacing = Units.inchesToMeters(48);
//        public static Translation2d autoGP1 = new Translation2d(
//                autoGPX,
//                autoGP_Yoffset+autoGPSpacing*0
//        );
//
//        public static Translation2d autoGP2 = new Translation2d(
//                autoGPX,
//                autoGP_Yoffset+autoGPSpacing*1
//        );
//
//        public static Translation2d autoGP3 = new Translation2d(
//                autoGPX,
//                autoGP_Yoffset+autoGPSpacing*2
//        );
//
//        public static Translation2d autoGP4 = new Translation2d(
//                autoGPX,
//                autoGP_Yoffset+autoGPSpacing*3
//        );
//
//        public static Translation2d pickupOffset = new Translation2d(DriveConstants.halfRobotLength, 0.0);

    }



    public static double blueAllianceScoreAngle = -0.5*Math.PI;
    public static double redAllianceScoreAngle = 0.5*Math.PI;


    /**
     * Use DriveToPoint for the first piece
     */
//    public static Pose2d cableGuardStartPoseBlue = new Pose2d(
//            DriveConstants.robotCentertoWall,
//            FieldConstants.BlueAlliance.ConeC1YLocation,
//            new Rotation2d(blueAllianceScoreAngle)
//    );
//
//    public static Pose2d retrieveGP1cableGuardBlue = new Pose2d(
//            AutoPieces.autoGP1.minus(AutoPieces.pickupOffset),
//            cableGuardStartPoseBlue.getRotation()
//    );
//
//
//
//    public static Pose2d scoreGP1_Cube_cableGuardBlue = new Pose2d(
//            DriveConstants.robotCentertoWall+DriveConstants.scoreApproachBuffer,
//            FieldConstants.BlueAlliance.CubeC2YLocation,
//            new Rotation2d(blueAllianceScoreAngle)
//    );


//    public List<Pose2d> scoreGP1CableGuardCubePath = List.of(
//            retreiveGP1cableGuard,
//            new Pose2d()
//    );

    public static double retrieveGP2AngleBlue = Units.degreesToRadians(225);
    public static Translation2d retrieveGP2Spacing = new Translation2d(
                        DriveConstants.halfRobotLength, DriveConstants.halfRobotLength
    );


//    public static Pose2d retrieveGP2SwitchpointCableGuard =
//            new Pose2d(5.5, scoreGP1_Cube_cableGuardBlue.getY(),
//                    new Rotation2d(retrieveGP2AngleBlue));
//
//    public static List<Pose2d> retrieveGP2CableGuardBlue = List.of(
//            new Pose2d(retrieveGP2SwitchpointCableGuard.getTranslation().minus(
//                    new Translation2d(0.1,0.0)),
//                    scoreGP1_Cube_cableGuardBlue.getRotation()),
//            retrieveGP2SwitchpointCableGuard,
//            new Pose2d(AutoPieces.autoGP2.minus(retrieveGP2Spacing), new Rotation2d(retrieveGP2AngleBlue))
//    );

//    public static List<Pose2d> scoreGP2CableGuardBlue = List.of(
//            retrieveGP2CableGuardBlue.get(2),
//            retrieveGP2CableGuardBlue.get(1),
//            scoreGP1_Cube_cableGuardBlue
//    );




    /*
    Freelane Paths
     */

//    public static Pose2d freelaneStartPose = new Pose2d(
//            DriveConstants.robotCentertoWall,
//            FieldConstants.BlueAlliance.ConeC9YLocation,
//            new Rotation2d(blueAllianceScoreAngle)
//    );
//
//    public static Pose2d pickupGP1freelane = new Pose2d(
//            AutoPieces.autoGP4.minus(AutoPieces.pickupOffset),
//            new Rotation2d(blueAllianceScoreAngle)
//    );
//
//    public static Pose2d scoreGP1freelane = new Pose2d(
//            DriveConstants.robotCentertoWall+DriveConstants.scoreApproachBuffer,
//            FieldConstants.BlueAlliance.CubeC8YLocation,
//            new Rotation2d(blueAllianceScoreAngle)
//    );



    public static double freelaneRetrieveGP2Angle = Units.degreesToRadians(-45);
    public static Translation2d freelaneRetrieveGP2Spacing = new Translation2d(
            DriveConstants.halfRobotLength, -DriveConstants.halfRobotLength
    );


//    public static Pose2d scoreGP1freelaneswitchpoint =
//            new Pose2d(5.5, scoreGP1freelane.getY(),
//                    new Rotation2d(freelaneRetrieveGP2Angle));



//    public static List<Pose2d> retrieveGP2freelane = List.of(
//            new Pose2d(scoreGP1freelaneswitchpoint.getTranslation(),
//                    new Rotation2d(blueAllianceScoreAngle)),
//            scoreGP1freelaneswitchpoint,
//            new Pose2d(AutoPieces.autoGP3.minus(freelaneRetrieveGP2Spacing), new Rotation2d(freelaneRetrieveGP2Angle))
//    );
//
//    public static List<Pose2d> scoreGP2freelane = List.of(
//            retrieveGP2freelane.get(2),
//            retrieveGP2freelane.get(1),
//            scoreGP1freelane
//    );
//
//
//
//
//









}


