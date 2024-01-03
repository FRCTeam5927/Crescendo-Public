//package frc.robot.commands.Drivetrain.Auto.Vision;
//
//import com.pathplanner.lib.PathPlannerTrajectory;
//import frc.robot.SubsystemContainer;
//import frc.robot.commands.Drivetrain.Auto.Odometry.FollowOdometerTrajectoryCommand;
//import frc.robot.subsystems.Drivetrain.DriveSubsystem;
//import frc.robot.subsystems.FusionEngine.FusedOdometer;
//
//public class FollowVisionTrajectoryCommand extends FollowOdometerTrajectoryCommand {
//
//    FusedOdometer _fieldGPS = SubsystemContainer.get_fieldGPS();
//    public FollowVisionTrajectoryCommand(DriveSubsystem dt, PathPlannerTrajectory traj) {
//        super(dt, traj);
//        _rposSupplier = _fieldGPS::get_fieldAbsolutePose;
//        _speedsConsumer = dt::setFieldAbsoluteChassisSpeeds;
//
//    }
//}
