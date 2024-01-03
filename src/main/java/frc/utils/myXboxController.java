package frc.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static java.lang.Math.abs;

public class myXboxController extends XboxController {

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public myXboxController(int port) {
        super(port);
    }

    public myXboxController(XboxController xboxController) {
        super(xboxController.getPort());


    }

//    public double

    public double getLeftXQuartic() {
        double LeftXvalue = applyDeadband(getLeftX());
        double LeftXsquared = LeftXvalue * LeftXvalue * LeftXvalue * LeftXvalue;

        if (LeftXvalue < 0) {
            return -LeftXsquared;
        } else {
            return LeftXsquared;
        }



    }

    public double getLeftYQuartic() {
        double LeftYvalue = applyDeadband(getLeftY());
        double LeftYsquared = LeftYvalue * LeftYvalue * LeftYvalue * LeftYvalue;

        if (LeftYvalue < 0) {
            return -LeftYsquared;
        } else {
            return LeftYsquared;
        }

    }

    public double getRightXQuartic() {
        double RightXvalue = applyDeadband(getRightX());
        double RightXvalueSquared = RightXvalue * RightXvalue * RightXvalue * RightXvalue;
        if (RightXvalue < 0) {
            return -RightXvalueSquared;
        } else {
            return RightXvalueSquared;
        }

    }


    public double getRightYQuartic() {
        double RightYvalue = getRightY();
        double RightYsquared = RightYvalue * RightYvalue * RightYvalue * RightYvalue;

        if (RightYvalue < 0) {
            return -RightYsquared;
        } else {
            return RightYsquared;
        }

    }

    public double getLeftTriggerAxisSquared() {
        double LeftTriggervalue = getLeftTriggerAxis();
        double LeftTriggersquared = LeftTriggervalue * LeftTriggervalue;

        if (LeftTriggervalue < 0) {
            return -LeftTriggersquared;
        } else {
            return LeftTriggersquared;
        }

    }

    public double getRightTriggerAxisSquared() {
        double RightTriggervalue = getRightTriggerAxis();
        double RightTriggersquared = RightTriggervalue * RightTriggervalue;

        if (RightTriggervalue < 0) {
            return -RightTriggersquared;
        } else {
            return RightTriggersquared;
        }

    }

    public boolean getDpadUp() {
        double UpPOV = this.getPOV();
        if (UpPOV == 0) {
            return true;
        } else {
            return false;
        }

    }

    public boolean getDpadRight() {
        double RightPOV = this.getPOV();
        if(RightPOV == 90) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getDpadDown() {
        double DownPOV = this.getPOV();
        if(DownPOV == 180) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getDpadLeft() {
        double LeftPOV = this.getPOV();
        if(LeftPOV == 270) {
            return true;
        } else {
            return false;
        }
    }

    public double applyDeadband(double value) {
        final double kDeadband = 0.05;
        if (abs(value) > kDeadband) {
            if (value > 0.0) {
                return (value - kDeadband) / (1.0 - kDeadband);
            } else {
                return (value + kDeadband) / (1.0 - kDeadband);
            }
        } else {
            return 0.0;
        }
    }

    public double getdummyleftX() {
        return SmartDashboard.getNumber("Autonomous Left X", 0);
    }

    public double getdummyleftY() {
        return SmartDashboard.getNumber("Autonomous Left Y" , 0);
    }

    public double getdummyrightX() {
        return SmartDashboard.getNumber("Autonomous Right X", 0);
    }

    public double getdummyrightY() {
        return SmartDashboard.getNumber("Autonomous Right Y", 0);
    }

//    public boolean getDpadUpPressed() {
//        if(getDpadUp()) {
//
//        }
//
//        this.getPOV()
//    }

}