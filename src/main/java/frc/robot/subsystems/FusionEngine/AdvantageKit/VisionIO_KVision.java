package frc.robot.subsystems.FusionEngine.AdvantageKit;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Odometry.VisionConstants;

import java.util.EnumSet;
import java.util.Objects;

import static frc.robot.Constants.Odometry.VisionConstants.visTabKey;

public class VisionIO_KVision extends SubsystemBase implements VisionIO {
    double[] visionData = new double[]{0,0,0,0,0,0,0,0,0,0};
    double lastTimestamp = 0.0;
    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    DoubleArrayEntry visEntry = networkTableInstance.getTable("Vision").getDoubleArrayTopic("Debug").getEntry(visionData);
//    TimestampedDoubleArray

    public VisionIO_KVision() {

    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionData = visEntry.get();
        double lastChange = visEntry.getLastChange();
        inputs.isDataNew = lastChange != lastTimestamp;
        lastTimestamp = lastChange;


        inputs.xMeters = visionData[0];
        inputs.yMeters = visionData[1];
        inputs.angleRad = visionData[2];
        inputs.timestamp = lastTimestamp;

        inputs.tagDist = visionData[3]/100;
        inputs.isOneTag = visionData[4] == 1;




    }




}
