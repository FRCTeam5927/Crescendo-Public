package frc.robot.libs.GamepieceSelector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.libs.Commands.GPSigaller.signalerSubsystem;


public class SignalCubeCommand extends Command {

    signalerSubsystem signaler;
    public SignalCubeCommand(signalerSubsystem signaler) {
        this.signaler = signaler;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(signaler);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        signaler.runCubeLight(true);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        signaler.runCubeLight(false);
    }
}
