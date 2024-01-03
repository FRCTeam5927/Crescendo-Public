package frc.robot.subsystems.Sensors;

public interface BaseIMU {
    double getYaw();
    double getYawRadPS();
    double getFusedHeading();
    void setYaw(double heading);
    void setFusedYaw(double heading);
}
