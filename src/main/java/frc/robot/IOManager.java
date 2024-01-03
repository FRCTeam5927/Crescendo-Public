package frc.robot;

import frc.robot.subsystems.Drivetrain.AdvantageKit.*;
import frc.robot.subsystems.FusionEngine.AdvantageKit.VisionIO;
import frc.robot.subsystems.FusionEngine.AdvantageKit.VisionIO_KVision;
import frc.robot.subsystems.FusionEngine.AdvantageKit.VisionIO_SIM;
import frc.robot.subsystems.Sensors.AdvantageKit.GyroIO;
import frc.robot.subsystems.Sensors.AdvantageKit.GyroIO_Pigeon;
import frc.robot.subsystems.Sensors.AdvantageKit.GyroIO_SIM;

public class IOManager {

    static boolean IS_REPLAY = false;


    private static final SwerveModuleIO[] modules = new SwerveModuleIO[4];;

    static {
        if(Robot.isReal()) {
            modules[0] = new SwerveModuleIOMK3(0);
            modules[1] = new SwerveModuleIOMK3(1);
            modules[2] = new SwerveModuleIOMK3(2);
            modules[3] = new SwerveModuleIOMK3(3);

        } else {
            if(IS_REPLAY) {
                modules[0] = new SwerveModuleIOMK3(0) {};
                modules[1] = new SwerveModuleIOMK3(1) {};
                modules[2] = new SwerveModuleIOMK3(2) {};
                modules[3] = new SwerveModuleIOMK3(3) {};
            } else {
                modules[0] = new SwerveModuleIOSIM_Akit();
                modules[1] = new SwerveModuleIOSIM_Akit();
                modules[2] = new SwerveModuleIOSIM_Akit();
                modules[3] = new SwerveModuleIOSIM_Akit();

            }
        }
    }

    public static SwerveModuleIO getSwerveModule(int index) {
        return modules[index];
    }

    private static final GyroIO gyroIO;


    static {
        if(Robot.isReal()) {
            gyroIO = new GyroIO_Pigeon();
        } else {
            if(IS_REPLAY) {
                gyroIO = new GyroIO() {};

            } else {
                gyroIO = new GyroIO_SIM();

            }
        }
    }

    public static GyroIO getGyroIO() {
        return gyroIO;
    }

    private static VisionIO visionIO;

    static {
        if(Robot.isReal()) {
            visionIO = new VisionIO_KVision();
        } else {
            if(IS_REPLAY) {
                visionIO = new VisionIO() {};
            } else {
                visionIO = new VisionIO_SIM();
            }
        }
    }

    public static VisionIO getVisionIO() {
        return visionIO;
    }

}
