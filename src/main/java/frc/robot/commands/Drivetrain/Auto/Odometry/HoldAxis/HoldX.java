package frc.robot.commands.Drivetrain.Auto.Odometry.HoldAxis;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint.DriveToPoint;
import frc.robot.libs.DSHelper;

public class HoldX extends DriveToPoint {

    DoubleSupplier jSupplier;
    protected double targetX;

    public HoldX(DoubleSupplier jSupplier, double targetX) {
        super(new Pose2d());
        this.jSupplier = jSupplier;
        _xRenderer = this::renderXJoy;
        this.targetX = targetX;
    }


    protected double renderXJoy() {
        double joy = MathUtil.applyDeadband(jSupplier.getAsDouble(), DriveConstants.BaseDriveRate);
        if(DSHelper.getAlliance() == DriverStation.Alliance.Red) {
            joy*=-1;
        }
        double speed = DriveConstants.kMaxSpeedMetersPerSecond*joy;
        return speed;
    }

    @Override
    public void execute() {
        _goalPose = new Pose2d(0.0, targetX, new Rotation2d());
        super.execute();
    }

}