//package frc.robot.commands.Drivetrain.Auto.Odometry;
//
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.DebugConstants;
//import frc.robot.subsystems.Drivetrain.DriveSubsystem;
//import static frc.robot.Constants.Drivetrain.DriveConstants.*;
//
//import java.util.function.Consumer;
//import java.util.function.Supplier;
//
//
//
//public class FollowOdometerTrajectoryCommand extends Command {
//    DriveSubsystem _drivetrain;
//    PPHolonomicDriveController driveController = new PPHolonomicDriveController(
//            new PIDController(9,0.1,0),//PIDConstants.xtracP,PIDConstants.xtracP,PIDConstants.xtracP),
//            new PIDController(9,0.1,0),//PIDConstants.ytracP,PIDConstants.ytracI,PIDConstants.ytracD),
//            new PIDController(6,0,0)//PIDConstants.rtracP,PIDConstants.rtracI,PIDConstants.rtracD)
//    );
//    protected Supplier<Pose2d> _rposSupplier;
//    protected Consumer<ChassisSpeeds> _speedsConsumer;
//    PathPlannerTrajectory _traj;
//    Timer _trajtimer;
//    double duration;
//    PathPlannerTrajectory.PathPlannerState finalState;
//    Pose2d finalPose;
//    boolean isStarted;
//
//    public FollowOdometerTrajectoryCommand(DriveSubsystem dt, PathPlannerTrajectory traj) {
//        _drivetrain = dt;
//        _traj = traj;
//        duration = _traj.getTotalTimeSeconds();
//        finalState = _traj.getEndState();
//        finalPose = finalState.poseMeters;
//        _speedsConsumer = dt::setRawChassisSpeeds;
//        _rposSupplier = dt::getPose;
//        if(DebugConstants.debugTrajectories) {
//            SmartDashboard.putNumber("TrajDuration", duration);
//        }
//
//        driveController.setTolerance(new Pose2d(
//                0.02,
//                0.02,
//                Rotation2d.fromDegrees(1)
//        ));
//        _trajtimer = new Timer();
//        addRequirements(dt);
//    }
//
//    @Override
//    public void initialize() {
//        if(DebugConstants.debugTrajectories) {
//            SmartDashboard.putNumber("TrajStage", 0.0);
//        }
//        _trajtimer.stop();
//        _trajtimer.reset();
//    }
//
//    @Override
//    public void execute() {
//
//        _trajtimer.start();
//        double secElapsed = _trajtimer.get();
//
//        if(DebugConstants.debugTrajectories) {
//            SmartDashboard.putNumber("TrajStage", 1.0);
//            SmartDashboard.putNumber("TrajcurrentTime", secElapsed);
//        }
//        followPPTrajectory(_traj, secElapsed);
//    }
//
//    public void followPPTrajectory(PathPlannerTrajectory trajectory, double currentTime) {
//        Trajectory.State wpiState = trajectory.sample(currentTime);
//        PathPlannerTrajectory.PathPlannerState pathPlannerState = (PathPlannerTrajectory.PathPlannerState) wpiState;
//
//        ChassisSpeeds driveSpeeds = driveController.calculate(
//                _rposSupplier.get(),
//                pathPlannerState
//        );
//
//
//        Pose2d currentCommandPose = pathPlannerState.poseMeters;
//        Pose2d actualPose = _rposSupplier.get();
//
//        if(DebugConstants.debugTrajectories) {
//            SmartDashboard.putNumber("Traj_ErrorX", actualPose.getX()-currentCommandPose.getX());
//            SmartDashboard.putNumber("Traj_ErrorY", actualPose.getY()-currentCommandPose.getY());
//
//            SmartDashboard.putString("TrajcurrentSpeeds", driveSpeeds.toString());
//            SmartDashboard.putBoolean("Traj isOnTrack", driveController.atReference());
//        }
//
//
//        _speedsConsumer.accept(driveSpeeds);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return _trajtimer.get() >= duration;//_drivetrain.isatPose(finalPose) || _trajtimer.get() >= duration;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        if(DebugConstants.debugTrajectories) {
//            SmartDashboard.putNumber("TrajStage", 2.0);
//        }
//        _trajtimer.stop();
//        _trajtimer.reset();
//    }
//}
