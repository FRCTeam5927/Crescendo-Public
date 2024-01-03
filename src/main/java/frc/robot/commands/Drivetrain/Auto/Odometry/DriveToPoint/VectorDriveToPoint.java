package frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.SubManager;
import frc.robot.libs.Commands.customCommandBase;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class VectorDriveToPoint extends customCommandBase {
//    boolean
    protected ProfiledPIDController magtrackController = new ProfiledPIDController(
            DriveConstants.PIDConstants.magtracP,
            DriveConstants.PIDConstants.magtracI,
            DriveConstants.PIDConstants.magtracD,
            DriveConstants.PIDConstants.baseXYConstraints
    );

    protected ProfiledPIDController rotTrackController = new ProfiledPIDController(
            DriveConstants.PIDConstants.rtracP,
            DriveConstants.PIDConstants.rtracI,
            DriveConstants.PIDConstants.rtracD,
            DriveConstants.PIDConstants.baseRotConstraints
    );
    protected Supplier<Pose2d> _poseSupplier;
    protected Consumer<ChassisSpeeds> _speedsConsumer;

    protected SwerveDriveController driveSubsystem = SubManager.get_drivetrain();
    protected Pose2d _goalPose;

    public VectorDriveToPoint(Pose2d goalPose) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveSubsystem);
        _goalPose = goalPose;
        _poseSupplier = driveSubsystem::getPose;
        _speedsConsumer = driveSubsystem::setChassisSpeeds;

        magtrackController.setTolerance(DriveConstants.PIDConstants.magtracTolerance);
//        magtrackController.setTolerance(DriveConstants.PIDConstants.magtracTolerance);

        rotTrackController.setTolerance(DriveConstants.PIDConstants.rtracTolerance);
        rotTrackController.enableContinuousInput(0, 2*Math.PI);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void onFirstLoop() {
        Pose2d pose = _poseSupplier.get();
        rotTrackController.reset(pose.getRotation().getRadians());
        Translation2d diff = pose.getTranslation().minus(_goalPose.getTranslation());
        magtrackController.reset(diff.getNorm());
    }

    public void trackToPoint(Pose2d goalPose) {
        Pose2d currentPose = _poseSupplier.get();
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d goalTranslation = goalPose.getTranslation();


        Translation2d deltaTranslation = currentTranslation.minus(goalTranslation);

        double distError = deltaTranslation.getNorm();
        double dirheading = deltaTranslation.getAngle().getRadians();

        double magOutput = magtrackController.calculate(distError, 0.0);
        double xoutput = magOutput*Math.cos(dirheading);
        double youtput = magOutput*Math.sin(dirheading);


        double currentAngle = currentPose.getRotation().getRadians();
        double holdAngle = goalPose.getRotation().getRadians();
        double rotOutput = rotTrackController.calculate(currentAngle, holdAngle);

        if(DebugConstants.debugTrajectories) {
            SmartDashboard.putNumber("VectorDistError", distError);
            SmartDashboard.putNumber("VectorMagOut", magOutput);
        }


        _speedsConsumer.accept(
                new ChassisSpeeds(
                        xoutput,
                        youtput,
                        rotOutput
                )
        );

//        driveSubsystem.fieldAbsoluteDrive(youtput, xoutput, rotOutput);
    }


    @Override
    public void execute() {
        super.execute();
        trackToPoint(
                _goalPose
        );
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
