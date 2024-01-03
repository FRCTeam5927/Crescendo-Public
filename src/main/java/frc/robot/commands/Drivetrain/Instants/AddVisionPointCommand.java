package frc.robot.commands.Drivetrain.Instants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SubManager;
import frc.robot.libs.MathLib;
import frc.robot.subsystems.FusionEngine.FusedOdometer;
import frc.robot.subsystems.Sensors.IMUSubsystem;


public class AddVisionPointCommand extends InstantCommand {

    public AddVisionPointCommand(Pose2d resetPose) {
        super(
                () -> SubManager.get_fieldGPS().overrideVision(resetPose)
        );

    }

}
