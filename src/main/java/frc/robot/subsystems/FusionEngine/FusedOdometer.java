package frc.robot.subsystems.FusionEngine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.Constants.Odometry.VisionConstants;
import frc.robot.Robot;
import frc.robot.SubManager;
import frc.robot.libs.DSHelper;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.libs.DiagnosticTable;
import frc.robot.libs.MathLib;
import frc.robot.subsystems.Sensors.IMUSubsystem;

import java.util.EnumSet;
import java.util.Objects;

import static frc.robot.Constants.Odometry.VisionConstants.*;

public class FusedOdometer extends SubsystemBase implements NetworkTable.TableEventListener {
    private final FusionCore fuser = new FusionCore();
    private Pose2d currentPose = new Pose2d();
    private final Field2d fieldDiagram = new Field2d();
    private int badVisionCount = 0;
    private int localVisionCount = 0;
    private final LinearFilter disabledRotationFilter = LinearFilter.movingAverage(20);
    DiagnosticTable dtab;

    LatencyCompensator lcomp = new LatencyCompensator(500);

    public FusedOdometer() {
        SmartDashboard.putData("Fused Field", fieldDiagram);
        NetworkTableInstance.getDefault().getTable(visTabKey).addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate) , this);
        disabledRotationFilter.reset();
        dtab = new DiagnosticTable("FusedOdometer");
    }


    double lastVisTime = 0.0;

    public double calculateLatency() {
        return 0.0;
    }

    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        if(event.is(NetworkTableEvent.Kind.kTimeSync)) {
            dtab.putBoolean("sawTimeSync", true);
        }
        if (Objects.equals(key, VisionConstants.monokey)) {
            NetworkTableInstance instance = NetworkTableInstance.getDefault();
//            instance.getServerTimeOffset();
            ConnectionInfo[] conns = instance.getConnections();
//            dtab.putString("sminf", conns[0]);
//            instance.

            dtab.putNumber("connlength", conns.length);
            if(instance.getServerTimeOffset().isPresent()) {
                dtab.putNumber("sto", instance.getServerTimeOffset().getAsLong());
            }

//            instance.addTimeSyncListener()
//            TimestampedDoubleArray darr = event.valueData;

            lastVisTime = Timer.getFPGATimestamp()-calculateLatency();

            TimeSyncEventData syncData = event.timeSyncData;

//            double transitTime = syncData.rtt2/1000000.0;
//            double timeOffset = syncData.serverTimeOffset/1000000.0;
            double dataTimestamp = event.valueData.value.getServerTime()/1000000.0;



            double NTnow = NetworkTablesJNI.now()/1000000.0;

//            double latency = NTnow-(dataTimestamp-timeOffset-transitTime);

            dtab.putNumber("NTnow", NTnow);
//            dtab.putNumber("latency", latency);

            double[] entryData = event.valueData.value.getDoubleArray();
            Pose2d fusedPose = filterVision(entryData);
            overrideVision(fusedPose);

            dtab.putString("fusedPose", fusedPose.toString());
//            dtab.putString("");
            fieldDiagram.setRobotPose(fusedPose);
//            injectVision(new Pose2d(
//                    entryData[0],
//                    entryData[1],
//                    Rotation2d.fromDegrees(entryData[2])
//            ));

            cacheVision(new Pose2d(
                    entryData[0],
                    entryData[1],
                    new Rotation2d(entryData[2])
            ));
        }
    }

    boolean hasNewData = false;
    Pose2d cachedVision = new Pose2d();
    private void cacheVision(Pose2d pose) {
        hasNewData = true;
        cachedVision = pose;
    }

    private Pose2d retreiveCachedVision() {
        hasNewData = false;
        return cachedVision;
    }



    private Translation2d filterBySpeed(Translation2d position) {
        Pose2d currPose2d = get_fieldAbsolutePose();
        ChassisSpeeds speedsCommmanded = SubManager.get_drivetrain().getFABSSpeeds();
        double currentTime = Timer.getFPGATimestamp()-SubManager.get_drivetrain().getOldPoseTimestamp();
        double maxTranslationX = (speedsCommmanded.vxMetersPerSecond * currentTime)  + VisionConstants.visiontackThreshold;
        double maxTranslationY = (speedsCommmanded.vyMetersPerSecond * currentTime)  + VisionConstants.visiontackThreshold;
        boolean isXOK = Math.abs(position.getX()-currPose2d.getX()) < maxTranslationX;
        boolean isYOK = Math.abs(position.getY()-currPose2d.getY()) < maxTranslationY;
        boolean isRotationOK = speedsCommmanded.omegaRadiansPerSecond < VisionConstants.rotationSpeedtoIgnoreVision;
        if(isXOK&&isYOK&&isRotationOK) {
            localVisionCount = 0;
            return position;
        } else {
            badVisionCount++;
            localVisionCount++;
            if(localVisionCount >3) {
                return  position;
            }
            return null;
        }
    }
    Pose2d lastOdoPose = new Pose2d();
    ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    @Override
    public void periodic() {
        Pose2d OdometerPose;

        if(Robot.isReal()) {
            OdometerPose = get_frelOdometerPose();
        } else {
            OdometerPose = get_frelOdometerPose();//VisionSpoofer.getOdometerPosition();
        }




        SwerveDriveController drv = SubManager.get_drivetrain();

        lastChassisSpeeds = MathLib.calculateTranslationVelocity(
                OdometerPose.getTranslation(),
                lastOdoPose.getTranslation(),
                drv.getOldPoseTimestamp()-drv.getOldOldPoseTimestamp()
        );

        lcomp.logState(OdometerPose, lastChassisSpeeds, SubManager.get_drivetrain().getOldPoseTimestamp());

//        dtab.putString;

        if(hasNewData) {
            Pose2d odoAtVision = null;
            try {
                 odoAtVision = lcomp.getAtTime(lastVisTime);

            } catch (NullPointerException e) {
                e.printStackTrace();
            } finally {
                if(odoAtVision == null) {
                    double pause = 0.009;
                } else {
                    Pose2d cachedVision = retreiveCachedVision();

                    dtab.putString("interpolatedTranslation", odoAtVision.toString());
                    dtab.putString("odotrans", OdometerPose.toString());

                    fuser.overrideLastOdometry(
                            odoAtVision.getTranslation()
                    );
                    fuser.registerVisionUpdate(
                            cachedVision.getTranslation()
                    );

                }
            }


        }



        fuser.registerOdometerLocation(OdometerPose.getTranslation());
        currentPose = new Pose2d(
                fuser.xLocation,
                fuser.yLocation,
                new Rotation2d(IMUSubsystem.getInstance().getFieldFusedYaw())
        );

        fieldDiagram.setRobotPose(
                currentPose.getX(),
                currentPose.getY(),
                Rotation2d.fromRadians(IMUSubsystem.getInstance().getDSrelfusedYaw())
        );
        if(DebugConstants.debugFuser) {
            dtab.putNumber("Fused Yaw (DS_REL)", Units.radiansToDegrees(IMUSubsystem.getInstance().getDSrelfusedYaw()));
            dtab.putNumber("Fused Yaw DIFF", Units.radiansToDegrees(IMUSubsystem.getInstance().getDSrelfusedYaw())-IMUSubsystem.getInstance().getYaw());
            dtab.putString("FUSEDPOSE", currentPose.toString());
            dtab.putString("ODOPOSE", OdometerPose.toString());
            dtab.putNumber("badVisionCount", badVisionCount);
            SmartDashboard.putData(fieldDiagram);

            lastOdoPose = OdometerPose;
        }
    }

    public Pose2d get_fieldAbsolutePose() {
        return currentPose;
    }

    public Pose2d get_frelOdometerPose() {
        Pose2d dtPose = SubManager.get_drivetrain().getPose();
        DriverStation.Alliance currentAlliance = DSHelper.getAlliance();
        double currentRotation = IMUSubsystem.getInstance().getFieldFusedYaw();//-dtPose.getRotation().getRadians();

        Rotation2d finalRot;

        if(Robot.isSimulation()) {
            finalRot = dtPose.getRotation();
        } else {
            finalRot = new Rotation2d(currentRotation);
        }

        return new Pose2d(
                MathLib.flipIfRed(dtPose.getX()),
                MathLib.flipIfRed(dtPose.getY()),
                finalRot
        );
    }

//    Timer odo

    public Pose2d filterVision(double[] visDat) {

        Translation2d vtrans = new Translation2d(visDat[0],visDat[1]);

        double distanceToTag = visDat[3]/100;
        boolean isOneTag = visDat[4] == 1.0;
        double filteredTagDist = MathUtil.clamp(distanceToTag, VisionConstants.closeDist, VisionConstants.farDist);
        double rangeRatio = filteredTagDist/VisionConstants.rangeDist;
//        Pose2d currentPose = new Pose2d(
//
//        )

        Pose2d lastPose = get_fieldAbsolutePose();

        double deltaX = Math.abs(visDat[0] - lastPose.getX());
        dtab.putNumber("Xerror", deltaX);
        double deltaY = Math.abs(visDat[1] - lastPose.getY());
        dtab.putNumber("Yerror", deltaX);


        Pose2d visionPose = new Pose2d(
                vtrans,
                Rotation2d.fromDegrees(visDat[2])
        );

        dtab.putString("vtrans", vtrans.toString());


//        dtab.putNumber("FusedX");

        if(deltaX > visiontackThreshold) {
            return visionPose;
        }

        if(deltaY > visiontackThreshold) {
            return visionPose;
        }

//        if(isOneTag) {
//            double visweightdist = (VisionConstants.oneTagVisWeightFar-VisionConstants.oneTagVisWeightClose) * rangeRatio;
////            double visweightSpeed =


            double speedRatioX = Math.abs(lastChassisSpeeds.vxMetersPerSecond/ DriveConstants.kMaxSpeedMetersPerSecond);
            double speedRatioY = Math.abs(lastChassisSpeeds.vxMetersPerSecond/ DriveConstants.kMaxSpeedMetersPerSecond);

            double weightXSpeed = speedRatioX*VisionConstants.oneTagVisWeightSpeedRange+oneTagVisWeightFast;
            double weightYSpeed = speedRatioY*VisionConstants.oneTagVisWeightSpeedRange+oneTagVisWeightFast;


            double weightedX =
                    (oneTagVisWeightClose*visionPose.getX())+
                    (oneTagOdoWeightClose*currentPose.getX());

            double weightedY =
                    (oneTagVisWeightClose*visionPose.getY())+
                    (oneTagOdoWeightClose*currentPose.getY());

            dtab.putNumber("xerr_2", weightedX-lastPose.getX());

            dtab.putString("fTrans", new Translation2d(weightedX,weightedY).toString());


            return new Pose2d(
                    weightedX,
                    weightedY,
                    visionPose.getRotation()
            );


//        } else {
//            return visionPose;
//        }


    }
    public void injectVision(Pose2d pose) {
        Pose2d diffPose = pose.relativeTo(currentPose);

        boolean xThresh = Math.abs(diffPose.getX()) > VisionConstants.visiontackThreshold;
        boolean yThresh = Math.abs(diffPose.getY()) > VisionConstants.visiontackThreshold;

        Translation2d ftrans = filterBySpeed(pose.getTranslation());
        if(ftrans == null) return;
        Pose2d nvis = new Pose2d(ftrans,pose.getRotation());
        if(xThresh || yThresh)  {
            fuser.registerVisionUpdate(nvis.getX(), nvis.getY());
        }

        double visionAngle = pose.getRotation().getRadians();
        double yawDiff = Math.abs(visionAngle-IMUSubsystem.getInstance().getFieldFusedYaw());
        if(DriverStation.isDisabled()) {
            double visionAngleFiltered = disabledRotationFilter.calculate(visionAngle);
            IMUSubsystem.getInstance().updateVisionAngle(visionAngleFiltered);
            SubManager.get_drivetrain().setOdometryAngle(Rotation2d.fromDegrees(visionAngle));
        } else if(yawDiff > VisionConstants.visionRotationtack) {
            IMUSubsystem.getInstance().updateVisionAngle(visionAngle);
            SubManager.get_drivetrain().setOdometryAngle(Rotation2d.fromDegrees(visionAngle));
        }

    }

    public void overrideVision(Pose2d pose) {
        IMUSubsystem.getInstance().updateVisionAngle(pose.getRotation().getRadians());
        SubManager.get_drivetrain().setOdometryAngle(new Rotation2d(IMUSubsystem.getInstance().getDSrelfusedYaw()));
        fuser.registerVisionUpdate(pose.getTranslation());

    }







}
