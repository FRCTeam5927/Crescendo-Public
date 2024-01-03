package frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.SubManager;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToPoint extends Command {
    protected TrapezoidProfile.Constraints ytracConstraints = DriveConstants.PIDConstants.baseXYConstraints;
    protected ProfiledPIDController ytrackController = new ProfiledPIDController(
            DriveConstants.PIDConstants.ytracP,
            DriveConstants.PIDConstants.ytracI,
            DriveConstants.PIDConstants.ytracD,
            ytracConstraints
    );

    protected TrapezoidProfile.Constraints xtracConstraints = DriveConstants.PIDConstants.baseXYConstraints;
    protected ProfiledPIDController xtrackController = new ProfiledPIDController(
            DriveConstants.PIDConstants.xtracP,
            DriveConstants.PIDConstants.xtracI,
            DriveConstants.PIDConstants.xtracD,
            xtracConstraints
    );


    protected TrapezoidProfile.Constraints rtracConstraints = DriveConstants.PIDConstants.baseRotConstraints;
    protected ProfiledPIDController rotTrackController = new ProfiledPIDController(
            DriveConstants.PIDConstants.rtracP,
            DriveConstants.PIDConstants.rtracI,
            DriveConstants.PIDConstants.rtracD,
            rtracConstraints
    );

    protected double xtracTolerance = DriveConstants.PIDConstants.xtracTolerance;
    protected double ytracTolerance = DriveConstants.PIDConstants.ytracTolerance;
    protected double rtracTolerance = DriveConstants.PIDConstants.rtracTolerance;

    protected Supplier<Pose2d> _poseSupplier;
    protected Consumer<ChassisSpeeds> _speedsConsumer;

    protected DoubleSupplier _xRenderer;
    protected DoubleSupplier _yRenderer;
    protected DoubleSupplier _rotRenderer;
    protected Pose2d currentPose;
    protected Supplier<ChassisSpeeds> _speedsSupplier;

    protected SwerveDriveController driveSubsystem = SubManager.get_drivetrain();
    protected Pose2d _goalPose;

    protected boolean kTrackDeadband = false;

    public DriveToPoint(Pose2d goalPose) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveSubsystem);
        _goalPose = goalPose;
        _poseSupplier = driveSubsystem::getPose;
        _speedsConsumer = driveSubsystem::setChassisSpeeds;
        _speedsSupplier = driveSubsystem::getDSRSpeeds;

        _xRenderer = this::renderXPID;
        _yRenderer = this::renderYPID;
        _rotRenderer = this::renderRotPID;

        xtrackController.setTolerance(xtracTolerance);

        ytrackController.setTolerance(ytracTolerance);

        rotTrackController.setTolerance(rtracTolerance);
        rotTrackController.enableContinuousInput(0, 2*Math.PI);

    }

    public DriveToPoint withTranslationConstraints(TrapezoidProfile.Constraints constraints) {
        xtrackController.setConstraints(constraints);
        ytrackController.setConstraints(constraints);
        return this;
    }

    public DriveToPoint withXConstraints(TrapezoidProfile.Constraints constraints) {
        xtrackController.setConstraints(constraints);

        return this;
    }

    public DriveToPoint withYConstraints(TrapezoidProfile.Constraints constraints) {
        ytrackController.setConstraints(constraints);
        return this;
    }

    public DriveToPoint withRotationConstraints(TrapezoidProfile.Constraints constraints) {
        rotTrackController.setConstraints(constraints);
        return this;
    }

    public DriveToPoint withTranslationController(ProfiledPIDController controller) {
        xtrackController = controller;
        ytrackController = controller;
        return this;
    }

    public DriveToPoint withXController(ProfiledPIDController controller) {
        xtrackController = controller;
        return this;
    }

    public DriveToPoint withYController(ProfiledPIDController controller) {
        ytrackController = controller;
        return this;
    }

    public DriveToPoint withRotationController(ProfiledPIDController controller) {
        rotTrackController = controller;
        return this;
    }

    public DriveToPoint withTranslationTolerance(double tolerance) {
        xtracTolerance = tolerance;
        ytracTolerance = tolerance;
        return this;
    }

    public DriveToPoint withRotationTolerance(double tolerance) {
        rtracTolerance = tolerance;
        return this;
    }

    public DriveToPoint withTracDeadband(boolean toDeadband) {
        kTrackDeadband = toDeadband;
        return this;
    }



    public void resetControllers() {
        Pose2d pose = _poseSupplier.get();
        SmartDashboard.putString("resetPose", pose.toString());
        rotTrackController.reset(pose.getRotation().getRadians());
        xtrackController.reset(pose.getX());
        ytrackController.reset(pose.getY());
    }

    @Override
    public void initialize() {


        resetControllers();

    }

    public void trackToPoint(Pose2d goalPose) {
        currentPose = _poseSupplier.get();
        _goalPose = goalPose;
        SmartDashboard.putString("goalPose", _goalPose.toString());
        SmartDashboard.putString("currentPose", currentPose.toString());
        double youtput = _yRenderer.getAsDouble();
//        SmartDashboard.putNumber("youtput", ytrackController.getVelocityError());
        SmartDashboard.putNumber("xConstraints", xtracConstraints.maxAcceleration);
        double xoutput = _xRenderer.getAsDouble();
        double rotOutput = _rotRenderer.getAsDouble();

        ChassisSpeeds commandSpeeds = new ChassisSpeeds(
                xoutput,
                youtput,
                rotOutput
        );
        SmartDashboard.putString("CommandSpeeds", commandSpeeds.toString());
        _speedsConsumer.accept(
                commandSpeeds
        );
    }

    protected double renderXPID() {
        double currentX = currentPose.getX();
        double targetX = _goalPose.getX();

        if(DebugConstants.driveToPoint) {
            SmartDashboard.putNumber("PointXERR", currentX-targetX);

        }

        if(Math.abs(getXError()) < xtracTolerance) {
            SmartDashboard.putBoolean("AAAAAAAAAAAAAAAAAAAAAAAAA", true);
            return 0.0;
        } else {
            SmartDashboard.putBoolean("AAAAAAAAAAAAAAAAAAAAAAAAAA", false);
            return xtrackController.calculate(currentX, targetX);
        }

    }


    protected double renderYPID() {
        double currentY = currentPose.getY();
        double targetY = _goalPose.getY();

        if(DebugConstants.driveToPoint) {
            SmartDashboard.putNumber("PointYCurrent", currentY);
            SmartDashboard.putNumber("PointYTARG", targetY);
            SmartDashboard.putNumber("PointYERR", currentY-targetY);
        }

        if(Math.abs(getYError()) < ytracTolerance) {
            return 0.0;
        } else {
            return ytrackController.calculate(currentY, targetY);
        }
    }

    protected double renderRotPID() {
        double currentRot = currentPose.getRotation().getRadians();
        double targetRot = _goalPose.getRotation().getRadians();

        if(DebugConstants.driveToPoint) {
            SmartDashboard.putNumber("PointRotERR", currentRot-targetRot);
            SmartDashboard.putNumber("currrentRot", currentRot);
            SmartDashboard.putNumber("targetRot", targetRot);
        }

        if(Math.abs(getRotError()) < rtracTolerance) {
            return 0.0;
        } else {
            return rotTrackController.calculate(currentRot, targetRot);
        }
    }

    protected double getXError() {
        Pose2d cpos = _poseSupplier.get();
        return cpos.getX()-_goalPose.getX();
    }

    protected boolean isAtPointX(double position, double tolerance) {
        Pose2d cpos = _poseSupplier.get();
        return Math.abs(cpos.getX()-position) < tolerance;
    }

    protected double getYError() {
        Pose2d cpos = _poseSupplier.get();
        return cpos.getY()-_goalPose.getY();
    }

    protected boolean isAtPointY(double position, double tolerance) {
        Pose2d cpos = _poseSupplier.get();
        return Math.abs(cpos.getY()-position) < tolerance;
    }

    protected double getRotError() {
        Pose2d cpos = _poseSupplier.get();
        return cpos.getRotation().getRadians()-_goalPose.getRotation().getRadians();
    }

    protected boolean isAtPointRot(double position, double tolerance) {
        Pose2d cpos = _poseSupplier.get();
        return Math.abs(cpos.getRotation().getRadians()-position) < tolerance;
    }

    @Override
    public void execute() {
        trackToPoint(
                _goalPose
        );
        if(DebugConstants.driveToPoint) {
            SmartDashboard.putBoolean("isFinished", isFinished());

        }
    }

    @Override
    public boolean isFinished() {
//        Pose2d currentPose = _poseSupplier.get();
//        return xtrackController.atSetpoint() &&
//                ytrackController.atSetpoint() &&
//                rotTrackController.atSetpoint();
        return Math.abs(getXError())<DriveConstants.PIDConstants.xtracTolerance &&
                Math.abs(getYError())<DriveConstants.PIDConstants.ytracTolerance &&
                Math.abs(getRotError())<DriveConstants.PIDConstants.rtracTolerance;
//        return false;
//        xtrackController.res
    }

    @Override
    public void end(boolean interrupted) {

    }
}
