package frc.robot.subsystems.FusionEngine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Odometry.VisionConstants;
import frc.robot.libs.DiagnosticTable;

public class VisionSpoofer extends SubsystemBase {

    public VisionSpoofer() {
        dtab.putNumber("xSpeed", 0.0);
        dtab.putNumber("ySpeed", 0.0);
    }

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable vtab = inst.getTable(VisionConstants.visTabKey);

    DoubleArrayEntry visent = vtab.getDoubleArrayTopic(VisionConstants.monokey).getEntry(new double[]{});
//    TimestampedDoubleArray
    int framecounter = 0;
    DiagnosticTable dtab = new DiagnosticTable("VisionSpoofer");
    ChassisSpeeds lastchassisSpeeds = new ChassisSpeeds();
    Pose2d lastOdo = new Pose2d();
    static Pose2d CurrentOdo = new Pose2d();

    private void updateNTvalues() {
        double newX = dtab.getNumber("xSpeed", 0.0);
        double newY = dtab.getNumber("ySpeed", 0.0);
        if(newX != lastchassisSpeeds.vxMetersPerSecond) lastchassisSpeeds.vxMetersPerSecond = newX;
        if(newY != lastchassisSpeeds.vyMetersPerSecond) lastchassisSpeeds.vyMetersPerSecond = newY;



    }
    public void advanceVision() {
        double advanceX = lastchassisSpeeds.vxMetersPerSecond*0.02;
        double advanceY = lastchassisSpeeds.vyMetersPerSecond*0.02;

        lastOdo = CurrentOdo;

        CurrentOdo = new Pose2d(
                CurrentOdo.getX() + advanceX,
                CurrentOdo.getY() + advanceY,
                new Rotation2d());


    }


    public Pose2d generateNoisyVision() {
        double noiseX = (Math.random() - 0.5) / 70.0;
        double noiseY = (Math.random() - 0.5) / 70.0;
        double noisyX = CurrentOdo.getX() + noiseX;
        double noisyY = CurrentOdo.getY() + noiseY;
        dtab.putNumber("Vision X", noisyX);
        dtab.putNumber("Vision Y", noisyY);
        return new Pose2d(
                noisyX, noisyY, new Rotation2d()
        );

    }

    public static Pose2d getOdometerPosition() {
        return CurrentOdo;
    }
    @Override
    public void periodic(){

        updateNTvalues();
        advanceVision();


        Pose2d newVis = generateNoisyVision();

//        framecounter++;
//        if(framecounter > 4) {
//            visent.set(new double[] {newVis.getX(),newVis.getY(),0.0,0.0,0.0,0.0,0.0});
//            framecounter = 0;
//        }
    }
 }
