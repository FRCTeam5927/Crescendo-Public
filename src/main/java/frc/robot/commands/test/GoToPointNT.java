package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.SubManager;
import frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint.DriveToPoint_Vision;
import frc.robot.libs.DiagnosticTable;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;


public class GoToPointNT extends DriveToPoint_Vision {
    DiagnosticTable diags;
    SwerveDriveController driveSubsystem;
    static Pose2d targPose = new Pose2d();
    public GoToPointNT() {
        super(targPose);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        driveSubsystem = SubManager.get_drivetrain();
        diags = new DiagnosticTable("DriveToPointNT");
        diags.putNumber("X_Position", 0.0);
        diags.putNumber("Y_Position", 0.0);
        diags.putNumber("Rotation_Angle_Degrees", 0.0);

        addRequirements();
    }

    @Override
    public void initialize() {
        targPose = new Pose2d(
                diags.getNumber("X_Position", 0.0),
                diags.getNumber("Y_Position", 0.0),
                Rotation2d.fromDegrees(diags.getNumber("Rotation_Angle_Degrees", 0.0))
        );
        _goalPose = targPose;
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

}
