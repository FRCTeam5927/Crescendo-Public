package frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOManager;
import frc.robot.subsystems.Sensors.AdvantageKit.GyroIO;
import frc.robot.subsystems.Sensors.AdvantageKit.GyroIOInputsAutoLogged;

public class PigeonSubsystem extends SubsystemBase implements BaseIMU {
    private final GyroIO _pigeon = IOManager.getGyroIO();
    private final GyroIOInputsAutoLogged _inputs = new GyroIOInputsAutoLogged();
    private double yawoffset = 0.0;
    private double fusedyawoffset = 0.0;

    public PigeonSubsystem() {

    }

    @Override
    public void periodic() {
        _pigeon.updateInputs(_inputs);
    }

    @Override
    public double getYaw() {
        return _inputs.yawPositionRad-yawoffset;
    }

    @Override
    public double getYawRadPS() {
        return _inputs.yawVelocityRadPerSec;
    }

    @Override
    public double getFusedHeading() {
        return _inputs.fusedHeading-fusedyawoffset;
    }

    @Override
    public void setYaw(double heading) {
        yawoffset = getYaw()-heading;
    }

    @Override
    public void setFusedYaw(double fusedYaw) {
        fusedyawoffset = getFusedHeading()-fusedyawoffset;
    }
}
