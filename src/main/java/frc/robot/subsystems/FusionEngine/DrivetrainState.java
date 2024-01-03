package frc.robot.subsystems.FusionEngine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DrivetrainState {

    private ChassisSpeeds robotSpeeds = new ChassisSpeeds();
    private Pose2d robotPose = new Pose2d();

    public DrivetrainState(Pose2d rpos, ChassisSpeeds rspeeds) {
        robotSpeeds = rspeeds;
        robotPose = rpos;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public void setRobotPose(Pose2d newPose) {
        robotPose = newPose;
    }

    public ChassisSpeeds getRobotSpeeds() {
        return robotSpeeds;
    }

    public void setRobotSpeeds(ChassisSpeeds rSpeeds) {
        this.robotSpeeds = rSpeeds;
    }
}
