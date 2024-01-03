package frc.robot.subsystems.FusionEngine.AdvantageKit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubManager;

public class VisionIO_SIM extends SubsystemBase implements VisionIO {
    double[] visionData = new double[]{0,0,0,0,0,0,0,0,0,0};
    double lastTimestamp = 0.0;
    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    DoubleArrayEntry visEntry = networkTableInstance.getTable("Vision").getDoubleArrayTopic("Debug").getEntry(visionData);
//    TimestampedDoubleArray

    public VisionIO_SIM() {

    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d fpose = SubManager.get_drivetrain().getPoseFABS();
        inputs.isDataNew = false;

        inputs.xMeters = fpose.getX();
        inputs.yMeters = fpose.getY();
        inputs.angleRad = fpose.getRotation().getRadians();
        inputs.timestamp = Timer.getFPGATimestamp();

    }




}
