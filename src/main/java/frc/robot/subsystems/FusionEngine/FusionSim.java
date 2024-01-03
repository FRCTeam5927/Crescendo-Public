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
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.DiagnosticTable;

public class FusionSim extends SubsystemBase {

    DiagnosticTable dtab = new DiagnosticTable("FusionSim");
    Field2d visField = new Field2d();
    Field2d fusedField = new Field2d();

    LinearSystem<N1, N1, N1> xSys = LinearSystemId.identifyVelocitySystem(1.0,0.2);
    LinearSystem<N1, N1, N1> ySys = LinearSystemId.identifyVelocitySystem(1.0,0.2);


    KalmanFilter<N1, N1, N1> xFilter =
            new KalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    xSys,
                    VecBuilder.fill(0.01),
                    VecBuilder.fill(0.01),
                    0.02
            );

    KalmanFilter<N1, N1, N1> yFilter =
            new KalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    ySys,
                    VecBuilder.fill(0.01),
                    VecBuilder.fill(0.01),
                    0.02
            );

    Matrix<N1, N1> xcmat = VecBuilder.fill(0.0);
    Matrix<N1, N1> ycmat = VecBuilder.fill(0.0);
    double lastodotimestamp = 0.0;
    double lastvistimestamp = 0.0;

//    Timer

    //    DoubleSupplier xJoy = new
    public FusionSim() {
        dtab.putNumber("xSpeed", 0.0);
        dtab.putNumber("ySpeed", 0.0);

        SmartDashboard.putData("VisionPose", visField);
        SmartDashboard.putData("FusedPoe", fusedField);
    }


    private ChassisSpeeds calculateTranslationVelocity(Translation2d vpos, Translation2d odopos, double period) {
        Translation2d diff = vpos.minus(odopos);
        Translation2d velocities = diff.div(period);

        return new ChassisSpeeds(
                velocities.getX(),
                velocities.getY(),
                0.0
        );
    }

    private Translation2d discretizeSpeed(ChassisSpeeds filteredSpeeds, double timeDiff) {
        return new Translation2d(
                filteredSpeeds.vxMetersPerSecond*timeDiff,
                filteredSpeeds.vyMetersPerSecond*timeDiff
        );
    }


    private void predictKalman(ChassisSpeeds projectSpeeds) {
        xcmat = VecBuilder.fill(projectSpeeds.vxMetersPerSecond);
        ycmat = VecBuilder.fill(projectSpeeds.vyMetersPerSecond);

        xFilter.predict(xcmat, 0.02);
        yFilter.predict(ycmat, 0.02);
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


    private void updateNTvalues() {
        double newX = dtab.getNumber("xSpeed", 0.0);
        double newY = dtab.getNumber("ySpeed", 0.0);
        if(newX != lastchassisSpeeds.vxMetersPerSecond) lastchassisSpeeds.vxMetersPerSecond = newX;
        if(newY != lastchassisSpeeds.vyMetersPerSecond) lastchassisSpeeds.vyMetersPerSecond = newY;



    }



    Pose2d lastOdo = new Pose2d();
    Pose2d CurrentOdo = new Pose2d();

    ChassisSpeeds lastchassisSpeeds = new ChassisSpeeds();

    public void kalmanLoop() {

    }
    @Override
    public void periodic() {
        Pose2d lastO = CurrentOdo;



        updateNTvalues();


        advanceVision();


        Pose2d noisyVision = generateNoisyVision();
        lastvistimestamp = Timer.getFPGATimestamp();
        double timeDiff = Math.abs(lastvistimestamp-lastodotimestamp);
        dtab.putNumber("timeDiff", timeDiff);


        ChassisSpeeds rawVisSpeeds = calculateTranslationVelocity(
                noisyVision.getTranslation(),
                lastOdo.getTranslation(), timeDiff
        );

        dtab.putString("rawVisSpeeds", rawVisSpeeds.toString());
        ChassisSpeeds filteredVisSpeeds = correctKalman(rawVisSpeeds);
        dtab.putString("filteredVisSpeeds", filteredVisSpeeds.toString());
        Translation2d filteredDelta = discretizeSpeed(filteredVisSpeeds, timeDiff);


        lastOdo = new Pose2d(
                lastOdo.getTranslation().plus(filteredDelta),
                noisyVision.getRotation()
        );
        lastodotimestamp = Timer.getFPGATimestamp();

        dtab.putNumber("Fused X", lastOdo.getX());

        dtab.putNumber("errorX", lastOdo.getX()-noisyVision.getX());

        dtab.putNumber("Vision X", noisyVision.getX());
        dtab.putNumber("Vision Y", noisyVision.getY());

        visField.setRobotPose(noisyVision);
        fusedField.setRobotPose(lastOdo);

        ChassisSpeeds calculatedSpeeds = calculateTranslationVelocity(CurrentOdo.getTranslation(), lastO.getTranslation(), 0.02);
        dtab.putString("predictorSpeeds", calculatedSpeeds.toString());
        predictKalman(calculatedSpeeds);
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
        double noiseX = (Math.random()-0.5)/68.0;
        double noiseY = (Math.random()-0.5)/50.0;
        double noisyX = CurrentOdo.getX()+noiseX;
        double noisyY = CurrentOdo.getY()+noiseY;
        dtab.putNumber("Vision X", noisyX);
        dtab.putNumber("Vision Y", noisyY);
        return new Pose2d(
                noisyX,noisyY,new Rotation2d()
        );
    }
}
