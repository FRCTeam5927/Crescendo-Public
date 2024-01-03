package frc.robot.libs.Commands.GPSigaller;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class signalerSubsystem extends SubsystemBase {

    Solenoid _cubeLight = new Solenoid(PneumaticsModuleType.REVPH, 1);
    Solenoid _whiteLight = new Solenoid(PneumaticsModuleType.REVPH, 2);
    Solenoid _coneLight = new Solenoid(PneumaticsModuleType.REVPH, 0);




    public signalerSubsystem() {

    }

    public void runCubeLight(boolean state) {
        _cubeLight.set(state);
    }

    public void runConeLight(boolean state) {
        _coneLight.set(state);
    }

    public void set_whiteLight(boolean state) {
        _whiteLight.set(state);
    }

    @Override
    public void periodic() {

    }

}

