package frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.Drivetrain.DriveConstants;
import frc.robot.libs.DSHelper;
import frc.robot.libs.DiagnosticTable;
import frc.robot.libs.MathLib;

import java.util.List;

import static java.lang.Math.abs;

public class PathFollower extends DriveToPoint {
    int increment = 0;
    Pose2d originalTarget;
    List<Pose2d> pose2dList;
    double finalX;
    double finalErrorX = Double.MAX_VALUE;
    double finalErrorY = Double.MAX_VALUE;
    int targetIndex = 0;
    int lastindex;
    Pose2d lastPose;

    Pose2d trackPose; 
    Pose2d stepPose;

    boolean xDirection;


    DiagnosticTable diags;

    public PathFollower(List<Pose2d> pose2dList) {
        super(pose2dList.get(0));
        this.pose2dList = pose2dList;

        diags = new DiagnosticTable(getName());




    }

    /**
     * This method would be used to accomodate re-planning of a PathPlanner Path
     */
    public void overridePathList(List<Pose2d> pathList) {
        this.pose2dList = pathList;
    }

    public List<Pose2d> getPathList() {
        return this.pose2dList;
    }


    @Override
    public void initialize() {
        lastindex = pose2dList.size()-1;
        lastPose = getStepPoseByIndex(lastindex);

        increment++;

        targetIndex = 0;
        trackPose = getTrackPoseByIndex(targetIndex);
        stepPose = getStepPoseByIndex(targetIndex);
        currentPose = _poseSupplier.get();


       if(DebugConstants.debugPathFollower) {
           diags.putNumber("startcount", increment);

           diags.putBoolean("IRAN", false);
           diags.putNumber("goal_pre", xtrackController.getGoal().position);

       }

       driveSubsystem.resetOdometry(pose2dList.get(0));
        resetControllers();
        setGoaltoTrackPose();


        double xError = (lastPose.getX()-stepPose.getX());
        if(xError == 0) {
            new PathFollower(null);
//            throw new Exception("You should probably use DriveToPoint for crossfield movement, finalX==startX");
        } else {
            xDirection = xError > 0.0;
        }

       if(DebugConstants.debugPathFollower) {

           diags.putNumber("GoalY_Velocity", ytrackController.getGoal().velocity);
           diags.putNumber("GoalY_Position", ytrackController.getGoal().position);
           diags.putNumber("GoalX_Velocity", xtrackController.getGoal().velocity);
           diags.putNumber("GoalX_Position", xtrackController.getGoal().position);

           diags.putBoolean("xDirection", xDirection);
           diags.putBoolean("IRAN", true);


           diags.putNumber("YERROR", getYError());

           diags.putNumber("XERROR", getXError());
       }


    }

    public void setGoaltoTrackPose() {
        Pose2d currentpose = _poseSupplier.get();

        xtrackController.setGoal(trackPose.getX());
        ytrackController.setGoal(trackPose.getY());
        rotTrackController.setGoal(trackPose.getRotation().getRadians());

    }

    public void resetAfterWaypoint() {
        setGoaltoTrackPose();
    }


    protected Pose2d getTrackPoseByIndex(int index) {
        Pose2d indexPose = getStepPoseByIndex(index);

        return new Pose2d(
            lastPose.getX(),
            indexPose.getY(),
            indexPose.getRotation()
        );

    }

    protected Pose2d getStepPoseByIndex(int index) {
        if(index>pose2dList.size()-1) {
            return lastPose;
        } else {
            Pose2d indexPose = pose2dList.get(index);
            
            if(DSHelper.getAlliance() == DriverStation.Alliance.Red) {
                return MathLib.translateForRed(indexPose);
            } else {
                return indexPose;
            }
        }


    }



    private void nextStep() {
        targetIndex++;
        if(targetIndex>lastindex) {
            targetIndex = lastindex;
        }
        trackPose=getTrackPoseByIndex(targetIndex);
        _goalPose=trackPose;
        stepPose=getStepPoseByIndex(targetIndex);

        resetAfterWaypoint();


    }

    @Override
    public void execute() {
        Pose2d cpos = _poseSupplier.get();

        finalErrorX = abs(trackPose.getX()-cpos.getX());
        finalErrorY = abs(trackPose.getY()-cpos.getY());

       double stepErrX = abs(stepPose.getX()-cpos.getX());
       double stepErrY = abs(stepPose.getY()-cpos.getY());

        boolean ispastX = cpos.getX() > stepPose.getX();

        if(xDirection && ispastX) {
            nextStep();
        } else if(!xDirection && !ispastX) {
            nextStep();
        }

        trackToPoint(trackPose);

        if(DebugConstants.debugPathFollower) {
            diags.putBoolean("isPastX", ispastX);


           diags.putString("currentPose", currentPose.toString());
           diags.putString("currentTrackPose", trackPose.toString());
           diags.putString("currentStepPose", stepPose.toString());

           diags.putNumber("TargetIndex", targetIndex);

           diags.putNumber("FinalErrorX", finalErrorX);
           diags.putNumber("FinalErrorY", finalErrorY);

           diags.putNumber("StepErrorX", stepErrX);
           diags.putNumber("StepErrorY", stepErrY);


           diags.putNumber("TrackErrorX", Math.abs(trackPose.getX()-cpos.getX()));
           diags.putNumber("TrackErrorY", Math.abs(trackPose.getY()-cpos.getY()));
           diags.putString("TrackError", trackPose.minus(currentPose).toString());



            diags.putNumber("GoalX_PositionEXEC", xtrackController.getGoal().position);
           diags.putNumber("GoalX_VelocityEXEC", xtrackController.getGoal().velocity);
           diags.putNumber("GoalX_VelErrorEXEC", xtrackController.getVelocityError());
           diags.putNumber("goalY_Velocity_EXEC", ytrackController.getGoal().velocity);

           diags.putBoolean("isFinished", isFinished());
        }




    }

    @Override
    public boolean isFinished() {
        return targetIndex >= lastindex &&
                finalErrorX < DriveConstants.PIDConstants.xtracTolerance &&
                finalErrorY < DriveConstants.PIDConstants.ytracTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        targetIndex = 0;
        resetControllers();
        setGoaltoTrackPose();
        super.end(interrupted);
    }


}
