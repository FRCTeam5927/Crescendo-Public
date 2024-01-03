package frc.robot.commands.Drivetrain.Auto.Vision.HoldAxis;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldData.FieldConstants;
import frc.robot.commands.Drivetrain.Auto.Odometry.HoldAxis.HoldX;

public class HoldX_Vision extends HoldX {

    public HoldX_Vision(DoubleSupplier jSupplier, double targetY) {
        super(jSupplier, targetY);
    }


    @Override
    public void execute() {
        _goalPose = new Pose2d(0.0, targetX, new Rotation2d(FieldConstants.getAllianceOffset()));
        super.execute();
    }
    
}