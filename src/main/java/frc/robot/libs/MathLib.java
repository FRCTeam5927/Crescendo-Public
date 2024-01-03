package frc.robot.libs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldData.FieldConstants;

import java.util.List;

public class MathLib {

    public static double signSafePower(double value, double exponent) {
            double poweredVal = Math.pow(value, exponent);
            if(value < 0 && exponent % 2 != 0) {
                poweredVal *= -1;
            }
            return poweredVal;
    }

    public static double[] allianceSafeXYTheta(double x, double y, double theta) {
        return new double[] {
                flipIfRed(x),
                flipIfRed(y),
                theta
        };
    }
    public static ChassisSpeeds flipXYIfRed(ChassisSpeeds speeds) {
        return new ChassisSpeeds(
                flipIfRed(speeds.vyMetersPerSecond),
                flipIfRed(speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond
        );
    }

    public static Pose2d translateDSRtoFABS(Pose2d pose) {

        return new Pose2d(
                flipIfRed(pose.getX()),
                flipIfRed(pose.getY()),
                pose.getRotation().plus(new Rotation2d(Math.PI/2.0))
        );
    }

    public static ChassisSpeeds rotateSpeeds(double rotateRadians, ChassisSpeeds speeds) {

        double dsang = rotateRadians;
        double cos = Math.cos(dsang);
        double sin = Math.sin(dsang);

        double xDSR = speeds.vxMetersPerSecond*cos + speeds.vyMetersPerSecond*sin;
        double yDSR = -speeds.vxMetersPerSecond*sin + speeds.vyMetersPerSecond*sin;

        return new ChassisSpeeds(
                xDSR, yDSR, speeds.omegaRadiansPerSecond
        );
    }

    /**
     *
     * @param i
     * the input value to flip if the robot is on red alliance.
     *
     * @return
     * the input value, flipped if the robot is on the red alliance.
     */
    public static double flipIfRed(double i) {
        if(DSHelper.getAlliance() == DriverStation.Alliance.Red) {
            return -i;
        } else {
            return i;
        }
    }

    public static ChassisSpeeds calculateTranslationVelocity(Translation2d vpos, Translation2d odopos, double period) {
        Translation2d diff = vpos.minus(odopos);
        Translation2d velocities = diff.div(period);

        return new ChassisSpeeds(
                velocities.getX(),
                velocities.getY(),
                0.0
        );


    }

    public static Translation2d discretizeSpeed(ChassisSpeeds filteredSpeeds, double timeDiff) {
        return new Translation2d(
                filteredSpeeds.vxMetersPerSecond*timeDiff,
                filteredSpeeds.vyMetersPerSecond*timeDiff
        );
    }


    public static double getNearestInArray(double value, double[] array) {
                    double diffVal = Double.MAX_VALUE;
        double target = 0.0;
        for (double val : array) {
            double cDiffVal = Math.abs(val - value);
            if(cDiffVal<diffVal) {
                diffVal = cDiffVal;
                target = val;
            }
        }
        return target;
    }

    public static Pose2d getClosestPoseInList(List<Pose2d> poseList, Pose2d ref) {
        return ref.nearest(poseList);
    }

    public static Pose2d allianceSafePose(Pose2d pose2d) {
        if(DSHelper.getAlliance() == DriverStation.Alliance.Red) {
            return translateForRed(pose2d);
        } else {
            return pose2d;
        }
    }
    public static Pose2d translateForRed(Pose2d pose2d) {
        return new Pose2d(
                translateToRed(pose2d.getTranslation()),
                new Rotation2d(2*Math.PI-(pose2d.getRotation().getRadians()))
        );
    }

    public static Translation2d translateToRed(Translation2d inTrans) {
        return new Translation2d(
                convertXToRed(inTrans.getX()),
                inTrans.getY()
        );
    }


    //TODO ADD FIELDCONSTANTS
    public static double convertXToRed(double x) {
        return 16-x;
    }
}
