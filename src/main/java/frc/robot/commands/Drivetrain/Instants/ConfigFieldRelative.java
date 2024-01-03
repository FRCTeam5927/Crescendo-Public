package frc.robot.commands.Drivetrain.Instants;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SubManager;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;


public class ConfigFieldRelative extends InstantCommand {

    public ConfigFieldRelative(boolean isFieldRelative) {
        super(
                () -> SubManager.get_drivetrain().config_fieldRelative(isFieldRelative)
        );
    }

}
