package frc.robot.commands.Drivetrain.Auto.Odometry.HoldAxis;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.libs.MathLib;

import java.util.function.DoubleSupplier;

public class HoldY_Nearest extends HoldY {
    double[] targetArray;
    public HoldY_Nearest(DoubleSupplier jSupplier, double[] targetList) {
        super(jSupplier, 0.0);
        this.targetArray = targetList;
    }

    @Override
    public void execute() {
        Pose2d currentPose = _poseSupplier.get();
        targetY = MathLib.getNearestInArray(currentPose.getX(), targetArray);
        super.execute();
    }
}
