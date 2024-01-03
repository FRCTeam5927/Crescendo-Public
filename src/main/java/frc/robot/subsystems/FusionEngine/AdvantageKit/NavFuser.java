package frc.robot.subsystems.FusionEngine.AdvantageKit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.Constants.Odometry.VisionConstants;
import frc.robot.IOManager;
import frc.robot.Robot;
import frc.robot.SubManager;
import frc.robot.libs.DiagnosticTable;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.FusionEngine.FusionCore;
import frc.robot.subsystems.FusionEngine.LatencyCompensator;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Odometry.VisionConstants.*;
import static frc.robot.Constants.Odometry.VisionConstants.oneTagOdoWeightClose;

public class NavFuser extends SubsystemBase {
    VisionIO visionIO = IOManager.getVisionIO();
    VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    FusionCore fuser = new FusionCore();
    LatencyCompensator lcomp = new LatencyCompensator(500);

    LinearFilter disabledRotationFilter = LinearFilter.movingAverage(100);
    String logKey = "NavFuser/";
    DiagnosticTable dtab = new DiagnosticTable(logKey);


    public NavFuser() {

    }

    @Override
    public void periodic() {
        visionIO.updateInputs(inputs);

        SwerveDriveController dt = SubManager.get_drivetrain();
        Pose2d odoPose = dt.getPoseFABS();
        ChassisSpeeds odoSpeeds = dt.getFABSSpeeds();

        lcomp.logState(odoPose, odoSpeeds, dt.getOldPoseTimestamp());



        if(inputs.isDataNew) {
            Translation2d vtrans_RAW = new Translation2d(inputs.xMeters, inputs.yMeters);
            double heading_RAW = inputs.angleRad;
            double timestamp = inputs.timestamp;


            if(DriverStation.isDisabled()) {
                double newYaw = disabledRotationFilter.calculate(heading_RAW);
                Logger.recordOutput(logKey+"filteredVisYaw");
                SubManager.get_pigeon().setFusedYaw(newYaw);
            }


            Pose2d odoAtVis;
            try {
                odoAtVis = lcomp.getAtTime(timestamp);
                fuser.overrideLastOdometry(odoAtVis.getTranslation());

                Translation2d filteredData = filterData(vtrans_RAW, odoAtVis.getTranslation());

                fuser.registerVisionUpdate(filteredData);



            } catch (NullPointerException e) {
                System.out.println("Bad Times");
            }
        }

    }

    public static double filterOverrideThreshold = 0.1;

    private Translation2d filterData(Translation2d vtrans, Translation2d odo) {
        Translation2d delta2D = vtrans.minus(odo);

        if(Math.abs(delta2D.getX()) > filterOverrideThreshold || Math.abs(delta2D.getY()) > filterOverrideThreshold) {
            return vtrans;
        } else {

            double distanceToTag = inputs.tagDist;
            boolean isOneTag = inputs.isOneTag;
            double filteredTagDist = MathUtil.clamp(distanceToTag, VisionConstants.closeDist, VisionConstants.farDist);
            double rangeRatio = filteredTagDist/VisionConstants.rangeDist;


            Pose2d currentFused = getFusedPose_FABS();



//            double speedRatioX = Math.abs(lastChassisSpeeds.vxMetersPerSecond/ DriveConstants.kMaxSpeedMetersPerSecond);
//            double speedRatioY = Math.abs(lastChassisSpeeds.vxMetersPerSecond/ DriveConstants.kMaxSpeedMetersPerSecond);
//
//            double weightXSpeed = speedRatioX*VisionConstants.oneTagVisWeightSpeedRange+oneTagVisWeightFast;
//            double weightYSpeed = speedRatioY*VisionConstants.oneTagVisWeightSpeedRange+oneTagVisWeightFast;



            double weightedX =
                    (oneTagVisWeightClose*vtrans.getX())+
                            (oneTagOdoWeightClose*currentFused.getX());

            double weightedY =
                    (oneTagVisWeightClose*vtrans.getY())+
                            (oneTagOdoWeightClose*currentFused.getY());


            return new Translation2d(
                    weightedX,
                    weightedY
            );

        }
    }

    public Pose2d getFusedPose_FABS() {
        return new Pose2d(
                fuser.xLocation,
                fuser.yLocation,
                new Rotation2d()
        );
    }

    public Pose2d getFusedPose_DSR() {
        return new Pose2d();
    }


}
