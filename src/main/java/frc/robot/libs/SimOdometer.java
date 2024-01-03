package frc.robot.libs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimOdometer extends SubsystemBase {

    Pose2d currentPose = new Pose2d();
    Field2d simfield = new Field2d();

    public SimOdometer() {
        SmartDashboard.putData("SimField", simfield);
    }

    public void update(ChassisSpeeds speeds) {
        SmartDashboard.putString("driveSpeeds",speeds.toString());
        currentPose = new Pose2d(
                currentPose.getX()+(speeds.vxMetersPerSecond*0.02),
                currentPose.getY()+(speeds.vyMetersPerSecond*0.02),
                currentPose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond*0.02))

                );
        simfield.setRobotPose(currentPose);
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

    @Override
    public void periodic() {

    }

}
