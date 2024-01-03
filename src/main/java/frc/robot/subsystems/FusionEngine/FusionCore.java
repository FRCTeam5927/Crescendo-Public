package frc.robot.subsystems.FusionEngine;

import edu.wpi.first.math.geometry.Translation2d;

public class FusionCore {

    public FusionCore() {
        xLocation = 0.0;
        yLocation = 0.0;
        previousOdometerXLocation = 0.0;
        previousOdometerYLocation = 0.0;
    }

    public double xLocation;
    public double yLocation;

    public double previousOdometerXLocation;
    public double previousOdometerYLocation;

    public void registerVisionUpdate(double x, double y){
        xLocation = x;
        yLocation = y;
    }

    public void registerVisionUpdate(Translation2d t) {
        registerVisionUpdate(t.getX(),t.getY());
    }

    public void registerOdometerLocation(double x, double y){
        xLocation += x - previousOdometerXLocation;
        previousOdometerXLocation = x;
        yLocation += y - previousOdometerYLocation;
        previousOdometerYLocation = y;
    }

    public void registerOdometerLocation(Translation2d t) {
        registerOdometerLocation(t.getX(),t.getY());
    }

    public void overrideLastOdometry(double x, double y) {
        previousOdometerXLocation = x;
        previousOdometerYLocation = y;
    }

    public void overrideLastOdometry(Translation2d t) {
        overrideLastOdometry(t.getX(),t.getY());
    }

}