package frc.robot.commands.Drivetrain.Instants;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;


public class ConfigRateLimit extends InstantCommand {

    public ConfigRateLimit(DriveSubsystem driveSubsystem, boolean toRateLimit) {
        super(
                () -> driveSubsystem.config_rateLimit(toRateLimit)
        );
    }

}
