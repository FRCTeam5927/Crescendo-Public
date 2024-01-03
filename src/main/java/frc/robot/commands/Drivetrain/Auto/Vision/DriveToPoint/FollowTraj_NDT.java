package frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Drivetrain.AutoConstants;
import frc.robot.libs.DiagnosticTable;

import java.util.ArrayList;
import java.util.List;

public class FollowTraj_NDT extends PathFollower_Vision {
    PathPlannerTrajectory plannerTraj;
    PathPlannerPath path;
    DiagnosticTable dtab;

    static List<Pose2d> plist = List.of(new Pose2d[]{
            new Pose2d(),
            new Pose2d(),
    });
    public FollowTraj_NDT(String pathName) {
        super(plist);
//        this.plannerTraj = plannerPath;
        this.path = PathPlannerPath.fromPathFile(pathName);
        this.plannerTraj = new PathPlannerTrajectory(this.path, _speedsSupplier.get());
        dtab = new DiagnosticTable("TrajFollowerNDT");
        reloadPath();

        initialize();
    }



    @Override
    public void initialize() {
//        reloadPath();
        publishPathNT();
        super.initialize();
    }

    public void reloadPath() {
//        Pose2d firstPose = path.getAllPathPoints().
//        SubsystemContainer.get_fieldGPS().overrideVision();
        Pose2d absPose = _poseSupplier.get();
        Pose2d ppPose = new Pose2d(
                absPose.getTranslation(),
                translate_FABSR_PPR(absPose.getRotation())
        );

//        path = path.replan(ppPose, _speedsSupplier.get());
//        plannerTraj = new PathPlannerTrajectory(path, _speedsSupplier.get());
        List<Pose2d> plist = convertToPoseList(plannerTraj);
        overridePathList(plist);
        PathConstraints pconstr = path.getGlobalConstraints();

        withTranslationConstraints(new TrapezoidProfile.Constraints(
                pconstr.getMaxVelocityMps(),
                pconstr.getMaxAccelerationMpsSq()
        ));

        withRotationConstraints(new TrapezoidProfile.Constraints(
                Units.rotationsToRadians(pconstr.getMaxAngularVelocityRps()),
                Units.rotationsToRadians(pconstr.getMaxAngularAccelerationRpsSq())
        ));

    }

    public static List<Pose2d> convertToPoseList(PathPlannerTrajectory traj) {
        List<Pose2d> poseList = new ArrayList<>();
        double step = 0.02;

        double duration  = traj.getTotalTimeSeconds();
        double steps = duration/step;

        for(int i =0; i<steps; i++) {
            double sampleTime = step*i;
            PathPlannerTrajectory.State timedSample = traj.sample(sampleTime);
            poseList.add(
                    new Pose2d(
                            timedSample.positionMeters,
                            timedSample.targetHolonomicRotation
//                            translate_PPR_FABSR(timedSample.targetHolonomicRotation)
                    )
            );
        }



        return poseList;
    }

    public void publishPathNT() {
        int cnt = 0;
        for(Pose2d cpos : getPathList()) {
            dtab.putString("PathStep "+cnt, cpos.toString());
            cnt++;
        }

    }

    public static Rotation2d translate_PPR_FABSR(Rotation2d dsrel) {
        return new Rotation2d(
                dsrel.getRadians()+AutoConstants.blueAllianceScoreAngle
        );
    }

    public static Rotation2d translate_FABSR_PPR(Rotation2d fabsr) {
        return new Rotation2d(
                fabsr.getRadians()-AutoConstants.blueAllianceScoreAngle
        );
    }
}
