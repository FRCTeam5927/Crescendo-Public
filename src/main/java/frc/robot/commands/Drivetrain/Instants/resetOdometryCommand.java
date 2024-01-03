package frc.robot.commands.Drivetrain.Instants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SubManager;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;


public class resetOdometryCommand extends InstantCommand {


    public resetOdometryCommand(DriveSubsystem driveSubsystem, Pose2d resetPose) {
        super(
                () -> SubManager.get_drivetrain().resetOdometry(resetPose)
        );
    }

}
