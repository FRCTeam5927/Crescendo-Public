package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.utils.mathlib;

import java.util.function.DoubleSupplier;

public class TURBOSwerveDriveCommand extends Command {
    boolean ratelimitCache;
    DoubleSupplier _xmag;
    DoubleSupplier _ymag;
    DoubleSupplier _rotmag;

    SwerveDriveController _dsub;


    public TURBOSwerveDriveCommand(SwerveDriveController dsub, DoubleSupplier xmag, DoubleSupplier ymag, DoubleSupplier rotmag) {
        _dsub = dsub;
        addRequirements(dsub);


        _xmag = xmag;
        _ymag = ymag;
        _rotmag = rotmag;
    }


    @Override
    public void initialize(){
        ratelimitCache = _dsub.get_rateLimit();

        _dsub.config_rateLimit(false);

    }

    @Override
    public void execute() {
        _dsub.drive(
                mathlib.applyDriveDeadband(_xmag.getAsDouble()),
                mathlib.applyDriveDeadband(_ymag.getAsDouble()),
                mathlib.applyDriveDeadband(_rotmag.getAsDouble())* DriveConstants.TURBORotScalar,
                DriveConstants.TURBOrate
                );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _dsub.config_rateLimit(ratelimitCache);

    }
}
