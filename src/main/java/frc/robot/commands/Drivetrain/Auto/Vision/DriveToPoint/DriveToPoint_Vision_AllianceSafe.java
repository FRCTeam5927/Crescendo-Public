package frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.libs.MathLib;

public class DriveToPoint_Vision_AllianceSafe extends DriveToPoint_Vision{

    Pose2d originalGoal;
    public DriveToPoint_Vision_AllianceSafe(Pose2d goalPose) {
        super(goalPose);
        originalGoal = goalPose;
    }


    @Override
    public void execute() {
        _goalPose = MathLib.allianceSafePose(originalGoal);
        trackToPoint(_goalPose);

    }
}
