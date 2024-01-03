package frc.robot.subsystems.Sensors.AdvantageKit;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.util.Units;

public class GyroIO_Pigeon implements GyroIO {

    PigeonIMU _pigeon = new PigeonIMU(2);

    @Override
    public void updateInputs(GyroIOInputs inputs) {
//        inputs.rollPositionRad = Units.degreesToRadians(_pigeon.getRoll());
//        inputs.pitchPositionRad = Units.degreesToRadians(_pigeon.getPitch());
        inputs.yawPositionRad = Units.degreesToRadians(_pigeon.getYaw());

        inputs.fusedHeading = Units.degreesToRadians(_pigeon.getFusedHeading());

        double[] rawGyro = new double[] {0,0,0};
        _pigeon.getRawGyro(rawGyro);
//        inputs.rollVelocityRadPerSec = Units.degreesToRadians(rawGyro[0]);
//        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(rawGyro[1]);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(rawGyro[2]);


    }
}
