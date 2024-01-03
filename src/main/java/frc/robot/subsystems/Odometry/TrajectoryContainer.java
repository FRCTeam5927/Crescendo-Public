//package frc.robot.subsystems.Odometry;
//
////import com.pathplanner.lib.PathConstraints;
////import com.pathplanner.lib.PathPlanner;
////import com.pathplanner.lib.PathPlannerTrajectory;
////import com.pathplanner.lib.PathPoint;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Constants.Drivetrain.DriveConstants;
//import frc.robot.commands.Drivetrain.Auto.Odometry.DriveTo.DriveToPoint;
//import frc.robot.commands.Drivetrain.Auto.Odometry.PathStep;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class TrajectoryContainer {
//    private final static PathPlannerTrajectory forwards2m = PathPlanner.generatePath(
//            new PathConstraints(
//                    DriveConstants.PIDConstants.baseXYConstraints.maxVelocity,
//                    DriveConstants.PIDConstants.baseXYConstraints.maxAcceleration
//                    ),
//            List.of(
//                    new PathPoint[]{
//                            new PathPoint(
//                                    new Translation2d(0,0),
//                                    Rotation2d.fromDegrees(0),
//                                    Rotation2d.fromDegrees(0)),
//                            new PathPoint(
//                                    new Translation2d(2.0,0.0),
//                                    Rotation2d.fromDegrees(0),
//                                    Rotation2d.fromDegrees(-30)
//                            )
//                    }
//            )
//    );
//
//
//
//    public static PathPlannerTrajectory getForwards2m() {
//        return forwards2m;
//    }
//
//
//    private static PathPlannerTrajectory testGUIpath = PathPlanner.loadPath(
//            "testpath",
//            new PathConstraints(
//                    DriveConstants.PIDConstants.baseXYConstraints.maxVelocity*2,
//                    DriveConstants.PIDConstants.baseXYConstraints.maxAcceleration
//            ));
//
//    public static PathPlannerTrajectory getTestGUIpath() {
//        return testGUIpath;
//    }
//
//    private static PathPlannerTrajectory TwoPiece = PathPlanner.loadPath(
//            "twopiece",
//            new PathConstraints(
//                    DriveConstants.PIDConstants.baseXYConstraints.maxVelocity/2,
//                    DriveConstants.PIDConstants.baseXYConstraints.maxAcceleration/3
//            ));
//
//    public static PathPlannerTrajectory getTwoPiece() {
//        return TwoPiece;
//    }
//
//    private static PathPlannerTrajectory s_curve = PathPlanner.loadPath(
//            "s-curve",
//            new PathConstraints(
//                    DriveConstants.PIDConstants.baseXYConstraints.maxVelocity*2,
//                    DriveConstants.PIDConstants.baseXYConstraints.maxAcceleration
//            ));
//
//    public static PathPlannerTrajectory getS_curve() {
//        return s_curve;
//    }
//
//
//    public static SequentialCommandGroup constructPath(PathPlannerTrajectory trajectory) {
//        List<Trajectory.State> statelist = trajectory.getStates();
//        List<Pose2d> poseList = new ArrayList<>();
//
////        trajectory.sample()
//
//        for(Trajectory.State currentState : statelist) {
//            poseList.add(currentState.poseMeters);
//        }
//
//        int lastIndex = poseList.lastIndexOf(poseList);
//        Pose2d lastPose = poseList.get(lastIndex);
//        poseList.remove(lastIndex);
//
//        return constructPath(poseList, lastPose, DriveConstants.PIDConstants.nextStepDistance);
//    }
//
//    public static SequentialCommandGroup constructPath(List<Pose2d> pointsList, Pose2d finalPose, double shiftDistance) {
//        List<Command> stepList = new ArrayList<>();
//
//        for(Pose2d currentStep : pointsList) {
//            stepList.add(new PathStep(currentStep).withTranslationTolerance(shiftDistance).withRotationTolerance(2*Math.PI));
//        }
//
//        stepList.add(new DriveToPoint(finalPose));
//
//        Command[] CommandArray = (Command[]) stepList.toArray();
//
//        return new SequentialCommandGroup(
//                CommandArray
//        );
//    }
//}
