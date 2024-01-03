package frc.robot.commands.Drivetrain.Auto.Odometry.HoldAxis;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint.DriveToPoint;
import frc.robot.libs.DSHelper;

public class HoldY extends DriveToPoint {

    DoubleSupplier jSupplier;
    protected double targetY;

    public HoldY(DoubleSupplier jSupplier, double targetY) {
        super(new Pose2d());
        this.jSupplier = jSupplier;
        _xRenderer = this::renderXJoy;
        this.targetY = targetY;
    }


    protected double renderXJoy() {
        double joy = MathUtil.applyDeadband(jSupplier.getAsDouble(), DriveConstants.kDriveDeadband)*DriveConstants.BaseDriveRate;
        if(DSHelper.getAlliance() == DriverStation.Alliance.Blue) {
            joy*=-1;
        }
        double speed = DriveConstants.kMaxSpeedMetersPerSecond*joy;
        return speed;
    }

    @Override
    public void execute() {
        _goalPose = new Pose2d(0.0, targetY, new Rotation2d());
        super.execute();
    }

}