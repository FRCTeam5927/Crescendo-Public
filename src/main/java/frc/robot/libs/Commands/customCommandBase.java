package frc.robot.libs.Commands;

import edu.wpi.first.wpilibj2.command.Command;

public class customCommandBase extends Command {
    protected boolean firstRun = true;

    /**
     * onFirstLoop will trigger every time a command activates. Useful for an initialize routine in a command that will be run multiple times
     */
    public void onFirstLoop() {

    }
    @Override
    public void execute() {
        if(firstRun) {
            onFirstLoop();
            firstRun = false;
        }
    }

    @Override
    public void end(boolean interrrupted) {
        firstRun = false;
    }
}
