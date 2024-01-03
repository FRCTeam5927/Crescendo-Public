package frc.robot.subsystems.FusionEngine;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.Odometry.VisionConstants;
import frc.robot.SubManager;
import frc.robot.libs.DSHelper;
import frc.robot.libs.DiagnosticTable;
import frc.robot.libs.MathLib;
import frc.robot.subsystems.Sensors.IMUSubsystem;

import java.util.EnumSet;
import java.util.Objects;

import static frc.robot.Constants.Odometry.VisionConstants.visTabKey;

public class KalmanFuser extends SubsystemBase implements NetworkTable.TableEventListener {
    DiagnosticTable dtab = new DiagnosticTable("KalmanFuser");
    Field2d visField = new Field2d();
    Field2d fusedField = new Field2d();

    LinearSystem<N2, N1, N1> xSys = LinearSystemId.identifyPositionSystem(2.0,0.11);
    LinearSystem<N2, N1, N1> ySys = LinearSystemId.identifyPositionSystem(1.0,0.11);


    KalmanFilter<N2, N1, N1> xFilter =
            new KalmanFilter<>(
                    Nat.N2(),
                    Nat.N1(),
                    xSys,
                    VecBuilder.fill(0.5,52.0),
                    VecBuilder.fill(0.001),
                    0.02
            );

    KalmanFilter<N2, N1, N1> yFilter =
            new KalmanFilter<>(
                    Nat.N2(),
                    Nat.N1(),
                    ySys,
                    VecBuilder.fill(0.05,2.0),
                    VecBuilder.fill(0.01),
                    0.02
            );

    Matrix<N1, N1> xcmat = VecBuilder.fill(0.0);
    Matrix<N1, N1> ycmat = VecBuilder.fill(0.0);
    double lastodotimestamp = 0.0;
    double lastvistimestamp = 0.0;

    Pose2d fusedPosition = new Pose2d();
    Pose2d lastODO = new Pose2d();

    public KalmanFuser() {
        NetworkTableInstance.getDefault().getTable(visTabKey).addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate) , this);
        SmartDashboard.putData("FusedField", fusedField);
//        Odo
    }

    public void fuseVision(Translation2d vTrans) {
        dtab.putString("visionTranslation", vTrans.toString());
        dtab.putNumber("visionX", vTrans.getX());
        dtab.putNumber("visionY", vTrans.getY());


        double timeDiff = Timer.getFPGATimestamp()-lastodotimestamp;
//        dtab.putNumber("timeDiff", timeDiff);
//        ChassisSpeeds rawVisSpeeds = calculateTranslationVelocity(vTrans, fusedPosition.getTranslation(), timeDiff);
//        ChassisSpeeds filteredVisSpeeds = correctKalman(rawVisSpeeds);
//        dtab.putString("filteredSpeeds", filteredVisSpeeds.toString());
//        Translation2d discreteDelta = discretizeSpeed(filteredVisSpeeds, timeDiff);
//        dtab.putString("discreteDelta", discreteDelta.toString());

        Translation2d filteredTranslation = correctTranslationKalman(vTrans);
        dtab.putString("correctedTranslation", filteredTranslation.toString());
        dtab.putNumber("xError", filteredTranslation.getX()-vTrans.getX());

        boolean xThresh = Math.abs(filteredTranslation.getX()-vTrans.getX()) > 0.03;
        boolean yThresh = Math.abs(filteredTranslation.getY()-vTrans.getY()) > 0.03;


//        if(xThresh || yThresh) {
//            resetKalman(vTrans);
//        }

        fusedPosition = new Pose2d(
                filteredTranslation,
                new Rotation2d()
        );

//        fusedPosition = new Pose2d(
//                fusedPosition.getTranslation().plus(discreteDelta),
//                fusedPosition.getRotation()
//        );

        lastvistimestamp = Timer.getFPGATimestamp();
        odofuser.registerVisionUpdate(filteredTranslation.getX(), filteredTranslation.getY());
    }
    public Pose2d getFusedPosition() {
        return new Pose2d(
                odofuser.xLocation,
                odofuser.yLocation,
                new Rotation2d()
        );
    }

    FusionCore odofuser = new FusionCore();
    ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public void kalmanLoop() {
//        Pose2d currentOdoPose = get_frelOdometerPose();
        Pose2d currentOdoPose = VisionSpoofer.getOdometerPosition();
//        SubsystemContainer.get_drivetrain().resetOdometry(
//                new Pose2d(
//                        -fusedPosition.getX(),
//                        -fusedPosition.getY(),
//                        new Rotation2d()
//                )
//        );

        odofuser.registerOdometerLocation(currentOdoPose.getX(), currentOdoPose.getY());
        fusedPosition = new Pose2d(
                odofuser.xLocation,
                odofuser.yLocation,
                new Rotation2d()
        );


        double lastOdoTime = Timer.getFPGATimestamp();

//        if(lastvistimestamp>lastodotimestamp) {
//            lastodotimestamp = lastvistimestamp;
//        }

        resetKalman(fusedPosition.getTranslation());
        ChassisSpeeds calculatedSpeeds = MathLib.calculateTranslationVelocity(currentOdoPose.getTranslation(), lastODO.getTranslation(), 0.02);
//        ChassisSpeeds calculatedSpeeds = calculateTranslationVelocity(fusedPosition.getTranslation(), lastODO.getTranslation(), lastOdoTime-lastodotimestamp);

        dtab.putString("calcSpeeds", calculatedSpeeds.toString());
        lastSpeeds = calculatedSpeeds;
        predictKalman(calculatedSpeeds);
        lastODO = currentOdoPose;
        lastodotimestamp = lastOdoTime;

    }


    private void predictKalman(ChassisSpeeds projectSpeeds) {
        xcmat = VecBuilder.fill(projectSpeeds.vxMetersPerSecond);
        ycmat = VecBuilder.fill(projectSpeeds.vyMetersPerSecond);

        xFilter.predict(xcmat, 0.02);
        yFilter.predict(ycmat, 0.02);
    }

    private void resetKalman(Translation2d rtrans) {
        xFilter.setXhat(VecBuilder.fill(rtrans.getX(), lastSpeeds.vxMetersPerSecond));
        yFilter.setXhat(VecBuilder.fill(rtrans.getY(), lastSpeeds.vyMetersPerSecond));
    }

    private Translation2d correctTranslationKalman(Translation2d rawtrans) {
        xFilter.correct(xcmat, VecBuilder.fill(rawtrans.getX()));
        yFilter.correct(ycmat, VecBuilder.fill(rawtrans.getY()));

        return new Translation2d(
                xFilter.getXhat(0),
                yFilter.getXhat(0)
        );
    }

    private ChassisSpeeds correctKalman(ChassisSpeeds visionSpeeds) {
//        currentVec = ;
        dtab.putNumber("visSpeedX", visionSpeeds.vxMetersPerSecond);
        dtab.putNumber("visSpeedY", visionSpeeds.vyMetersPerSecond);

        xFilter.correct(xcmat, VecBuilder.fill(visionSpeeds.vxMetersPerSecond));
        yFilter.correct(ycmat, VecBuilder.fill(visionSpeeds.vyMetersPerSecond));

        double correctedX = xFilter.getXhat().get(0,0);
        double correctedY = yFilter.getXhat().get(0,0);
        dtab.putNumber("correctedX", correctedX);
        dtab.putNumber("correctedY", correctedY);
        return new ChassisSpeeds(correctedX, correctedY, 0.0);
    }

    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        if (Objects.equals(key, VisionConstants.monokey)) {
            double[] entryData = event.valueData.value.getDoubleArray();

            Translation2d vtrans = new Translation2d(
                    entryData[0],
                    entryData[1]
            );
            fuseVision(vtrans);
//            injectVision(new Pose2d(
//                    entryData[0],
//                    entryData[1],
//                    Rotation2d.fromDegrees(entryData[2])
//            ));
        }
    }

    @Override
    public void periodic() {
        kalmanLoop();
        Pose2d fusedPosition = getFusedPosition();

        if(DebugConstants.debugFuser) {
            dtab.putNumber("fusedX", fusedPosition.getX());
            dtab.putNumber("fusedY", fusedPosition.getY());
            fusedField.setRobotPose(fusedPosition);
        }
    }

    private static Pose2d get_frelOdometerPose() {
        Pose2d dtPose = SubManager.get_drivetrain().getPose();
        DriverStation.Alliance currentAlliance = DSHelper.getAlliance();
        double currentRotation = IMUSubsystem.getInstance().getFieldFusedYaw();//-dtPose.getRotation().getRadians();

        if(currentAlliance == DriverStation.Alliance.Red) {
            return new Pose2d(
                    -dtPose.getX(),
                    -dtPose.getY(),
                    new Rotation2d(currentRotation)
            );
        } else {
            return new Pose2d(
                    dtPose.getX(),
                    dtPose.getY(),
                    new Rotation2d(currentRotation)
            );
        }
    }


}
