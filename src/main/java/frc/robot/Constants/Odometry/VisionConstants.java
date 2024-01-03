package frc.robot.Constants.Odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Drivetrain.DriveConstants;

public class VisionConstants {
    public static double rotationSpeedtoIgnoreVision = DriveConstants.kMaxAngularSpeed;
    public static double visiontackThreshold = 0.025;

    public static double visionRotationtack = Units.degreesToRadians(3);
    public static final double minOneTagTransStdDev = 0.005;
    public static final double maxOneTagTransStdDev = 0.03;

    public static final double minMultiTagTransStdDev = 0.04;//0.004;
    public static final double maxMultiTagTransStdDev = 0.04;

    public static final double minOneTagRotStdDev = Units.degreesToRadians(0.23889);
    public static final double maxOneTagRotStdDev = Units.degreesToRadians(0.507738);

    public static final double minMultiTagRotStdDev = Math.PI/4;//Units.degreesToRadians(0.21236);
    public static final double maxMultiTagRotStdDev = 0.04;

    public static final Matrix<N3, N1> odometerWeights = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5));
    public static final Matrix<N3, N1> oneTagVisionWeightsMin = VecBuilder.fill(0.0, 0.0, Units.degreesToRadians(0));
    public static final Matrix<N3, N1> oneTagVisionWeightsMax = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(10));

    public static final Matrix<N3, N1> multiTagVisionWeightsMin = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(10));
    public static final Matrix<N3, N1> multiTagVisionWeightsMax = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(10));


    public static final double closeDist = 0.464;
    public static final double farDist = 2.98;

    public static final double oneTagVisWeightClose = 0.2;
    public static final double oneTagOdoWeightClose = 1-oneTagVisWeightClose;

    public static final double oneTagVisWeightFar = 0.0;

    public static final double oneTagVisWeightSlow = 0.0;
    public static final double oneTagVisWeightFast = 0.0;
    public static final double oneTagVisWeightSpeedRange = oneTagVisWeightSlow-oneTagVisWeightFast;



    public static final double mtagcloseDist = 0.735;
    public static final double mtagfarDist = 2.98;
    public static final double rangeDist = farDist-closeDist;
    public static final double mtagrangeDist = mtagfarDist-mtagcloseDist;

    public static final double kA = 0.0;
    public static final double kV = 0.0;

//    public static LinearSystem<N2, N1, N1> drivetrainX = LinearSystemId.identifyPositionSystem(kV,kA);
//    public static LinearSystem<N2, N1, N1> drivetrainY = LinearSystemId.identifyPositionSystem(kV,kA);
//    public static LinearSystem<N2, N1, N1> drivetrainRot = LinearSystemId.identifyPositionSystem(kV,kA);


    public static double nearvisionTransStdDev = 0.03;


//    public static KalmanFilter<N1,N1,N2> xfilter = new KalmanFilter<>(
//            Nat.N1(),
//            Nat.N1(),
//            drivetrainX,
//            VecBuilder.fill(0.03),
//            odometerWeights,
//            0.02
//    );


    public static final String visTabKey = "Vision/Debug";

    public static final String yawKey = "Yaw";
    public static final String rposKey = "Position";
    public static final String monokey = "TagDebug";
}
