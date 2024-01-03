package frc.robot.commands.Drivetrain.Auto.Vision.HoldAxis;

import frc.robot.SubManager;
import frc.robot.commands.Drivetrain.Auto.Odometry.HoldAxis.HoldY_Nearest;

import java.util.function.DoubleSupplier;

public class HoldY_Nearest_Vision extends HoldY_Nearest {


    public HoldY_Nearest_Vision(DoubleSupplier jSupplier, double[] targetList) {
        super(jSupplier, targetList);
        _poseSupplier = SubManager.get_fieldGPS()::get_fieldAbsolutePose;
        _speedsConsumer = SubManager.get_drivetrain()::setFABSSpeeds;
        _speedsSupplier = SubManager.get_drivetrain()::getFABSSpeeds;
    }
}
