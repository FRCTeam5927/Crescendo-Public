package frc.robot.commands.Drivetrain.Tuner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Odometry.VisionConstants;

import java.util.*;


public class gatherVisionStandardDeviationsCommand extends Command implements NetworkTable.TableEventListener  {

    NetworkTableInstance _networkTableInstance = NetworkTableInstance.getDefault();
    NetworkTable _visTab = _networkTableInstance.getTable("Vision/Debug");
    public gatherVisionStandardDeviationsCommand() {
        _visTab.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate) , this);
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }


    public static double calculateSD(List<Double> numArray)
    {
        double sum = 0.0, standardDeviation = 0.0;
        int length = numArray.size()-1;
        SmartDashboard.putNumber("length", length);

        for(double num : numArray) {
//            SmartDashboard.putNumber("num", num);
            sum += num;
        }

        double mean = sum/length;
//        SmartDashboard.putNumber("sum", sum);
//        SmartDashboard.putNumber("mean", mean);

        for(double num: numArray) {
            standardDeviation += Math.pow(num - mean, 2);
        }

        return Math.sqrt(standardDeviation/length);
    }

    List<Double> xHistory = new ArrayList<>();
    List<Double> yHistory = new ArrayList<>();
    List<Double> rHistory = new ArrayList<>();
    @Override
    public void end(boolean interrupted) {
        double xStdDev = calculateSD(xHistory);
        double yStdDev = calculateSD(yHistory);
        double rStdDev = calculateSD(rHistory);
        SmartDashboard.putNumber("StandardDeviation_X", xStdDev);
        SmartDashboard.putNumber("StandardDeviation_Y", yStdDev);
        SmartDashboard.putNumber("StandardDeviation_R", rStdDev);
        xHistory.clear();
        yHistory.clear();
        rHistory.clear();
    }



    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        SmartDashboard.putBoolean("isScheduled", isScheduled());
        if(isScheduled()) {
            if (Objects.equals(key, VisionConstants.monokey)) {
                double[] entrydata = event.valueData.value.getDoubleArray();
                SmartDashboard.putNumber("data", entrydata[0]);
                xHistory.add(entrydata[0]);
                yHistory.add(entrydata[1]);
                rHistory.add(entrydata[2]);
            }
        }

    }
}
