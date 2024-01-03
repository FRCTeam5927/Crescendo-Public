package frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Drivetrain.AutoConstants;
//import frc.robot.Constants.Drivetrain.AutoConstants;

import java.util.ArrayList;
import java.util.List;

public class FollowPath_NDT extends PathFollower_Vision {
    PathPlannerPath plannerPath;
    public FollowPath_NDT(PathPlannerPath plannerPath) {
        super(convertToPoseList(plannerPath));
        this.plannerPath = plannerPath;
    }


    @Override
    public void initialize() {
        Pose2d absPose = _poseSupplier.get();
        Pose2d ppPose = new Pose2d(
                absPose.getTranslation(),
                translate_FABSR_PPR(absPose.getRotation())
        );
        List<Pose2d> plist = convertToPoseList(plannerPath.replan(ppPose, _speedsSupplier.get()));
        overridePathList(plist);
        PathConstraints pconstr = plannerPath.getGlobalConstraints();
//        pconstr.
        withTranslationConstraints(new TrapezoidProfile.Constraints(
                pconstr.getMaxVelocityMps(),
                pconstr.getMaxAccelerationMpsSq()
        ));

        withRotationConstraints(new TrapezoidProfile.Constraints(
                Units.rotationsToRadians(pconstr.getMaxAngularVelocityRps()),
                Units.rotationsToRadians(pconstr.getMaxAngularAccelerationRpsSq())
        ));

        super.initialize();
    }


    public static List<Pose2d> convertToPoseList(PathPlannerPath path) {
//        path.
        List<Pose2d> poseList = new ArrayList<>();

        for(PathPoint pathPoint : path.getAllPathPoints()){
//            pathPoint.`
            poseList.add(
                    new Pose2d(
                            pathPoint.position,
                            translate_PPR_FABSR(pathPoint.holonomicRotation)
                    )
            );
        }

        return poseList;
    }

    public static Rotation2d translate_PPR_FABSR(Rotation2d dsrel) {
        return new Rotation2d(
                dsrel.getRadians()+ AutoConstants.blueAllianceScoreAngle
        );
    }

    public static Rotation2d translate_FABSR_PPR(Rotation2d fabsr) {
        return new Rotation2d(
                fabsr.getRadians()-AutoConstants.blueAllianceScoreAngle
        );
    }
}
