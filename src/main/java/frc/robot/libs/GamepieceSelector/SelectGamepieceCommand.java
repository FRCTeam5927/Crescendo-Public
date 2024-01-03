package frc.robot.libs.GamepieceSelector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.libs.GamePieceSelector;
import frc.robot.libs.GamepieceType;


public class SelectGamepieceCommand extends InstantCommand {

    public SelectGamepieceCommand(GamepieceType gamepieceType) {
        super(() -> GamePieceSelector.set(gamepieceType));
    }
}
