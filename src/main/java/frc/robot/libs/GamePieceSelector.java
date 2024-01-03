package frc.robot.libs;

import frc.robot.libs.GamepieceType;

public class GamePieceSelector {
    private static GamepieceType _selectedGamepiece = GamepieceType.Cone;

    public static void selectCone() {
        _selectedGamepiece = GamepieceType.Cone;
    }

    public static void selectCube() {
        _selectedGamepiece = GamepieceType.Cube;
    }

    public static void set(GamepieceType gamepieceType) {
        _selectedGamepiece = gamepieceType;
    }

    public static GamepieceType get() {
        return _selectedGamepiece;
    }
}
