package frc.robot;


import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import

// import frc.robot.subsystems.ArmMechanism2D;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.FusionEngine.FusedOdometer;
import frc.robot.subsystems.FusionEngine.KalmanFuser;
import frc.robot.subsystems.FusionEngine.VisionSpoofer;
import frc.robot.libs.Commands.GPSigaller.signalerSubsystem;
import frc.robot.subsystems.Sensors.PigeonSubsystem;

/**
 * This class should handle all subsystems and their setup
 */
public class SubManager extends SubsystemBase {
    /**
     * Place Subsystem objects and getters here
     */


    private static SwerveDriveController _drivetrain;
    private static  PigeonSubsystem _pigeon;
    private static FusedOdometer _fieldGPS;

    private static signalerSubsystem _signaler;
//    private static PigeonSubsystem _pigeon = new PigeonSubsystem();


    public static SwerveDriveController get_drivetrain() {
        return _drivetrain;
    }
    public static PigeonSubsystem get_pigeon() {
        return _pigeon;
    }

    public static FusedOdometer get_fieldGPS() {
        return _fieldGPS;
    }


    public static signalerSubsystem get_signaler() {
        return _signaler;
    }


    /**
     * Run init functions in the constructor
     */
    public static void initialize() {

        CommandScheduler.getInstance().registerSubsystem();
        _pigeon = new PigeonSubsystem();

        _drivetrain = new SwerveDriveController();
        _signaler = new signalerSubsystem();

        init_fieldGPS();


    }


    /**
     * MUST be called AFTER the drivetrain is initialized.
     */

    static VisionSpoofer visionSpoofer;
    private static void init_fieldGPS() {
        _fieldGPS = new FusedOdometer();
//        fsim = new FusionSim();
//        kfuser = new KalmanFuser();

//        if(Robot.isSimulation()) {
//            visionSpoofer = new VisionSpoofer();
//        }
            
    }




    DoubleLogEntry timelogger = new DoubleLogEntry(DataLogManager.getLog(), "Robot Loop Timestamps");
    DoubleLogEntry frametimelogger = new DoubleLogEntry(DataLogManager.getLog(), "Robot Loop Timings");
    DoubleLogEntry fpslogger = new DoubleLogEntry(DataLogManager.getLog(), "Robot Loop FPS");


    double prevFrameTime = Timer.getFPGATimestamp();
    /**
     * Place all periodic/diagnostic code here
     */
    static PathPlannerPath p = PathPlannerPath.fromPathFile("ScoreGP3");

    @Override
    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();
        double frameTime = currentTime-prevFrameTime;
        double fps = 1/frameTime;
        timelogger.append(currentTime);
        frametimelogger.append(frameTime);
        fpslogger.append(fps);



    }


}
