package frc.robot.libs.Commands.GPSigaller;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.libs.GamePieceSelector;
import frc.robot.libs.GamepieceType;

public class GamePieceCommand extends SelectCommand {


    public GamePieceCommand(Command coneCommand, Command cubeCommand) {
        super(
            Map.ofEntries(
                Map.entry(GamepieceType.Cone, coneCommand),
                Map.entry(GamepieceType.Cube, cubeCommand)
            ),
            GamePieceSelector::get
        );
    }
}
