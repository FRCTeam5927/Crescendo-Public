package frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class DriveToNearestPose extends DriveToPoint{
    List<Pose2d> poseList;
    public DriveToNearestPose(List<Pose2d> poseList) {
        super(new Pose2d());
        this.poseList = poseList;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = _poseSupplier.get();
        _goalPose = currentPose.nearest(poseList);
        super.initialize();
    }

}
