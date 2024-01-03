package frc.robot.commands.Drivetrain.Tuner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDriveTunerCommand extends Command {
    boolean fieldRelative;

    DoubleSupplier _xmag;
    DoubleSupplier _ymag;
    DoubleSupplier _rotmag;
    DoubleSupplier _throttlemag;

    SwerveDriveController _dsub;


    public SwerveDriveTunerCommand(SwerveDriveController dsub) {
        _dsub = dsub;
        addRequirements(dsub);

    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        _dsub.drive(
                2.0,
                2.0,
                0.0,
                0.0
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _dsub.drive(0.0,0.0,0.0,0.0);
    }
}
