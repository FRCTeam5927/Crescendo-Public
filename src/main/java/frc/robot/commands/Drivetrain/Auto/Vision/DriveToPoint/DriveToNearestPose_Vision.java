package frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.SubManager;
import frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint.DriveToNearestPose;

import java.util.List;

public class DriveToNearestPose_Vision extends DriveToNearestPose {
    public DriveToNearestPose_Vision(List<Pose2d> poseList) {
        super(poseList);
        _poseSupplier = SubManager.get_fieldGPS()::get_fieldAbsolutePose;
        _speedsConsumer = SubManager.get_drivetrain()::setFABSSpeeds;
        _speedsSupplier = SubManager.get_drivetrain()::getFABSSpeeds;
    }
}
