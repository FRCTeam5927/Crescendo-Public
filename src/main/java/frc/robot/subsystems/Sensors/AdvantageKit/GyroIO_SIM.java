package frc.robot.subsystems.Sensors.AdvantageKit;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.util.Units;
import frc.robot.SubManager;

public class GyroIO_SIM implements GyroIO {



    @Override
    public void updateInputs(GyroIOInputs inputs) {
//        inputs.rollPositionRad = Units.degreesToRadians(_pigeon.getRoll());
//        inputs.pitchPositionRad = Units.degreesToRadians(_pigeon.getPitch());
        inputs.yawPositionRad = SubManager.get_drivetrain().getPoseDSR().getRotation().getRadians();

        inputs.fusedHeading = SubManager.get_drivetrain().getPoseFABS().getRotation().getRadians();

//        inputs.rollVelocityRadPerSec = Units.degreesToRadians(rawGyro[0]);
//        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(rawGyro[1]);
        inputs.yawVelocityRadPerSec = SubManager.get_drivetrain().getRRSpeeds().omegaRadiansPerSecond;

    }
}
