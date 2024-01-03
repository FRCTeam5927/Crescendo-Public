package frc.robot.commands.Drivetrain.Instants;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;


public class ResetHeadingCommand extends InstantCommand {

    public ResetHeadingCommand(SwerveDriveController driveSubsystem) {
        super(driveSubsystem::resetHeading);
    }

}
