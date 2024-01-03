package frc.robot.commands.Drivetrain.Tuner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;


public class SwerveModuleAngleTunerCommand extends Command {
    private final SwerveDriveController driveSubsystem;
    private double targPos = 0.0;
    private boolean toTrack = false;

    public SwerveModuleAngleTunerCommand(SwerveDriveController driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.tuneSwerveModuleAngle();

    }

    @Override
    public void execute() {
        driveSubsystem.tuneSwerveModuleAngle();

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
