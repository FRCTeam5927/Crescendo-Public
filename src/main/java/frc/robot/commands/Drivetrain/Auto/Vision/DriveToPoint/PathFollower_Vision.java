package frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.SubManager;
import frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint.PathFollower;
import frc.robot.subsystems.FusionEngine.FusedOdometer;

import java.util.List;

import static java.lang.Math.abs;

public class PathFollower_Vision extends PathFollower {


    FusedOdometer _fieldGPS = SubManager.get_fieldGPS();
    public PathFollower_Vision(List<Pose2d> pose2dList) {
        super(pose2dList);
        _poseSupplier = _fieldGPS::get_frelOdometerPose;
        _speedsConsumer = driveSubsystem::setFABSSpeeds;
        _speedsSupplier = driveSubsystem::getFABSSpeeds;
    }
}
