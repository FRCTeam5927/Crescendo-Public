package frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.SubManager;
import frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint.DriveToPoint;
import frc.robot.subsystems.FusionEngine.FusedOdometer;

public class DriveToPoint_Vision extends DriveToPoint {
    FusedOdometer _fieldGPS = SubManager.get_fieldGPS();
    public DriveToPoint_Vision(Pose2d goalPose) {
        super(goalPose);
        _poseSupplier = _fieldGPS::get_frelOdometerPose;
        _speedsConsumer = driveSubsystem::setFABSSpeeds;
        _speedsSupplier = driveSubsystem::getFABSSpeeds;

        rotTrackController.setP(-rotTrackController.getP());
    }

//    @Override
//    public boolean isFinished() {
//        return xtrackController.atGoal()&&ytrackController.atGoal();
//    }
}
