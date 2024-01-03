package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivetrain.DriveConstants;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.utils.mathlib;

import static frc.robot.Constants.Drivetrain.OIConstants.kDriveDeadband;

public class SwerveDriveCommand extends Command {
    boolean fieldRelative;

    DoubleSupplier _xmag;
    DoubleSupplier _ymag;
    DoubleSupplier _rotmag;
    DoubleSupplier _throttlemag;

    SwerveDriveController _dsub;


    public SwerveDriveCommand(SwerveDriveController dsub, DoubleSupplier xmag, DoubleSupplier ymag, DoubleSupplier rotmag, DoubleSupplier throttlemag) {
        _dsub = dsub;
        addRequirements(dsub);

        _xmag = xmag;
        _ymag = ymag;
        _rotmag = rotmag;
        _throttlemag = throttlemag;
    }


    @Override
    public void initialize(){
        _dsub.resetHeading();
    }

    @Override
    public void execute() {
        _dsub.drive(
                mathlib.applyDriveDeadband(_xmag.getAsDouble()),
                mathlib.applyDriveDeadband(_ymag.getAsDouble()),
                mathlib.applyDriveDeadband(_rotmag.getAsDouble())* DriveConstants.BaseRotScalar,
                MathUtil.applyDeadband(_throttlemag.getAsDouble(), kDriveDeadband)
        );
        SmartDashboard.putNumber("DS_X", _xmag.getAsDouble());
//        IMUSubsystem.getInstance().getDSrelfusedYaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
