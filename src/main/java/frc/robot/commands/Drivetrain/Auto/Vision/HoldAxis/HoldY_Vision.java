package frc.robot.commands.Drivetrain.Auto.Vision.HoldAxis;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldData.FieldConstants;
import frc.robot.SubManager;
import frc.robot.commands.Drivetrain.Auto.Odometry.HoldAxis.HoldY;

public class HoldY_Vision extends HoldY {

    public HoldY_Vision(DoubleSupplier jSupplier, double targetY) {
        super(jSupplier, targetY);
        _poseSupplier = SubManager.get_fieldGPS()::get_fieldAbsolutePose;
        _speedsConsumer = driveSubsystem::setFABSSpeeds;
    }


    @Override
    public void execute() {
        currentPose = _poseSupplier.get();
        _goalPose = new Pose2d(0.0, targetY, new Rotation2d(FieldConstants.getAllianceOffset()));
        trackToPoint(_goalPose);
    }
    
}