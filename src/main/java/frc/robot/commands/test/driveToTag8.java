package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint.DriveToPoint_Vision;

public class driveToTag8 extends DriveToPoint_Vision {
    public driveToTag8() {
        super(new Pose2d(
                2.5274, 1.0716,
                new Rotation2d(-0.5*Math.PI)
        ));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
