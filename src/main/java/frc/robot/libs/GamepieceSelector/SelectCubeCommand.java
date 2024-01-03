package frc.robot.libs.GamepieceSelector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.libs.GamePieceSelector;


public class SelectCubeCommand extends InstantCommand {

    public SelectCubeCommand() {
        super(GamePieceSelector::selectCube);
    }
}
