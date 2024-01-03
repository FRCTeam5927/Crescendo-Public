package frc.robot.libs.BaseMechanism;

import com.revrobotics.CANSparkMaxLowLevel;

public class ControlModes {
    public enum PositionMode {
        RacePID,
        SmartMotion
    }

    public enum VelocityMode {
        Basic,
        Profiled
    }

    public enum PIDMethod {
        OnControllerPIDF,
        SoftwarePIDF,
        HybridPIDF,
        onControllerPIDSoftwareFF,
        SoftwarePIDHybridFF,
        /**
         *  OnControllerPID abd SoftwarePID set all feed-forward values to 0
         */
        OnControllerPID,
        SoftwarePID,
        /**
         *  OnControllerFF, SoftwareFF and HybridFF set all PID values to 0, ensuring only Feedforward values affect the output.
         *  Useful for tuning the FeedForward parameters
         */
        OnControllerFF,
        SoftwareFF,
        HybridFF


    }
}
