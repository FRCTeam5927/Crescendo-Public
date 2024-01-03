package frc.robot.subsystems.FusionEngine.AdvantageKit;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public double timestamp = 0.0;
        public boolean isDataNew = true;
        public double xMeters = 0.0;
        public double yMeters = 0.0;
        public double angleRad = 0.0;
        public boolean isOneTag = false;
        public double tagDist = 0.0;

    }


    public default void updateInputs(VisionIOInputs inputs) {}

}
