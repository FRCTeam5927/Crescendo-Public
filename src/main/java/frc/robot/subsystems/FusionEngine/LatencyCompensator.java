package frc.robot.subsystems.FusionEngine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.DiagnosticTable;

import java.util.*;

public class LatencyCompensator {

    int logLength;



    TreeMap<Double, DrivetrainState> statelog;
    int maxLength = 0;

    DiagnosticTable dtab = new DiagnosticTable("LatencyCompensator");
    public LatencyCompensator(int maxLength) {
        this.maxLength = maxLength;

        statelog = new TreeMap<>();

    }

    public void enforceLimit() {
        while(statelog.size() > maxLength) {
            Map.Entry<Double, DrivetrainState> firstEntry = statelog.firstEntry();
            statelog.remove(firstEntry.getKey());
        }
        SmartDashboard.putNumber("logsize", statelog.size());
    }
    public double calcDisplacement(double vel, double acc, double time) {
        return vel*time;//+(0.5*(acc*Math.pow(time, 2)));
    }


    public Pose2d getAtTime(double time) {
//        time
        enforceLimit();

        Map.Entry<Double, DrivetrainState> floorState;
        Map.Entry<Double, DrivetrainState> ceilingState;

        try {
            floorState = statelog.floorEntry(time);

            dtab.putString("floorState", floorState.getValue().getRobotPose().toString());

            ceilingState = statelog.ceilingEntry(time);

            dtab.putString("ceilState", ceilingState.getValue().getRobotPose().toString());


            ChassisSpeeds floorSpeeds = floorState.getValue().getRobotSpeeds();
            double floortime = floorState.getKey();
            dtab.putNumber("floortime", floortime);
            ChassisSpeeds ceilSpeeds = ceilingState.getValue().getRobotSpeeds();
            double ceiltime = ceilingState.getKey();

            double deltatime = time-floortime;
            dtab.putNumber("deltatime", deltatime);


            ChassisSpeeds accelerations = new ChassisSpeeds(
                    ceilSpeeds.vxMetersPerSecond-floorSpeeds.vxMetersPerSecond,
                    ceilSpeeds.vyMetersPerSecond-floorSpeeds.vyMetersPerSecond,
                    ceilSpeeds.omegaRadiansPerSecond-floorSpeeds.omegaRadiansPerSecond
            );

            //  ut + ½ at2 where u is velocity

            dtab.putNumber("fakeXD", floorSpeeds.vxMetersPerSecond*deltatime);

            double xDisplacement = calcDisplacement(floorSpeeds.vxMetersPerSecond, accelerations.vyMetersPerSecond, deltatime);

            dtab.putNumber("faxeXDP", xDisplacement);

            dtab.putString("floorSpeeds", floorSpeeds.toString());

            Pose2d deltaPose = new Pose2d(
                    xDisplacement,
                    calcDisplacement(floorSpeeds.vyMetersPerSecond, accelerations.vyMetersPerSecond, deltatime),
                    new Rotation2d(calcDisplacement(floorSpeeds.omegaRadiansPerSecond, accelerations.omegaRadiansPerSecond, deltatime))
            );

            dtab.putString("deltaPose", deltaPose.toString());
            Pose2d floorPose = floorState.getValue().getRobotPose();

            return new Pose2d(
                    floorPose.getTranslation().plus(deltaPose.getTranslation()),
                    floorPose.getRotation().plus(deltaPose.getRotation())
            );
        } catch (NullPointerException e) {
//            e.printStackTrace();
            return null;
        }



    }

    public void logState(Pose2d rpos, ChassisSpeeds rspeeds, double time) {
        statelog.put(time, new DrivetrainState(rpos, rspeeds));
        enforceLimit();
    }


}
