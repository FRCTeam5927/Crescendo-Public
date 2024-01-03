package frc.robot.libs.Commands;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.libs.DSHelper;

public class AllianceCommand extends SelectCommand {

    public AllianceCommand(Command blueCommand, Command redCommand) {
        super(
            Map.ofEntries(
                Map.entry(DriverStation.Alliance.Blue, blueCommand),
                Map.entry(DriverStation.Alliance.Red, redCommand)
            ),
            DSHelper::getAlliance
        );
    }
    
}
