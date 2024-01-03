package frc.robot.subsystems.Sensors;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.FieldData.FieldConstants;
import frc.robot.Constants.Odometry.VisionConstants;
import frc.utils.myPigeonIMU;
import java.util.EnumSet;
import java.util.Objects;


public class IMUSubsystem extends SubsystemBase implements NetworkTable.TableEventListener{
    private static boolean firstRead = true;
    private double lastIMUAngle = 0.0;
    private double fieldFusedAngle = 0.0;
    private double visAngle = 0.0;
    private static final IMUSubsystem INSTANCE = new IMUSubsystem();
    public static myPigeonIMU _pigeon = new myPigeonIMU(2);
    public static IMUSubsystem getInstance() {
        return INSTANCE;
    }


    public IMUSubsystem(){
        NetworkTableInstance.
                getDefault().
                getTable("Vision").
                addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate) , this);
        CommandScheduler.getInstance().registerSubsystem(this);
    }
    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        if (Objects.equals(key, VisionConstants.yawKey)) {
            updateVisionAngle(event.valueData.value.getDouble());
        }
    }

    public void updateVisionAngle(double visionAngle) {
        visAngle = visionAngle;
        fieldFusedAngle = visionAngle;
        lastIMUAngle = getfrelYaw();
    }

    public double getYaw(){
        return _pigeon.getYaw();
    }

    public double getYawRadians() {
        return Units.degreesToRadians(getYaw());
    }

    public double getfrelYaw() {
        double currentYaw = -getYawRadians();
        return currentYaw+ FieldConstants.getAllianceOffset();
    }

    public double getDSrelfusedYaw() {
        return -(getFieldFusedYaw()+ FieldConstants.getAllianceOffset());
    }

    public double getFieldFusedYaw() {
        return fieldFusedAngle;
    }

    public double getYawDPS() {
        return _pigeon.getYawDPS();
    }

    @Override
    public void periodic() {
        if (firstRead && DriverStation.isEnabled()) {
            fieldFusedAngle = FieldConstants.getAllianceOffset();
            firstRead = false;
        }
        fieldFusedAngle = visAngle + getfrelYaw() - lastIMUAngle;

        if(DebugConstants.debugIMU) {
            SmartDashboard.putNumber("Pigeon_Roll", _pigeon.getRoll());
            SmartDashboard.putNumber("Pigeon_RollDPS", _pigeon.getrollDPS());
            SmartDashboard.putBoolean("Pigeon_RollFalling", _pigeon.isLevelingROLL());
            SmartDashboard.putNumber("Pigeon_Pitch", _pigeon.getPitch());
            SmartDashboard.putNumber("Pigeon_Yaw", _pigeon.getYaw());
            SmartDashboard.putNumber("VDS_FUSEDYAW", Units.radiansToDegrees(getDSrelfusedYaw()));
        }
    }
}
