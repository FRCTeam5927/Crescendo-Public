package frc.utils;
import com.ctre.phoenix.sensors.PigeonIMU;

import static java.lang.Math.abs;

public class myPigeonIMU extends PigeonIMU  {
//    PigeonIMU
    double rolldps_Prev1 = 0;
    double rolldps_Prev2 = 0;
    double fallingthresh = -1/50.0;
    public myPigeonIMU(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public double getYaw() {
        double[] ypr = new double[3];
        this.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public double getYawDPS() {
        double[] ypr_DPS = new double[3];
        this.getRawGyro(ypr_DPS);
        return ypr_DPS[0];
    }

    public double getrollDPS() {
        double[] ypr_DPS = new double[3];
        this.getRawGyro(ypr_DPS);
        return ypr_DPS[2];
    }

    public boolean isLevelingROLL(){
        double rolldeg = applydeadband(abs(this.getRoll()));
        boolean falling1 = (rolldeg-rolldps_Prev1)<fallingthresh;
        boolean falling2 = (rolldeg-rolldps_Prev2)<fallingthresh;
        boolean abovethresh = rolldeg>7;
        rolldps_Prev2 = rolldps_Prev1;
        rolldps_Prev1 = rolldeg;
        return falling1&&falling2&&abovethresh;
    }

    private double applydeadband(double input){
        if(input >- 0.2&&input < 0.2) {
            return 0.0;
        }
        return input;
    }



    public void reset() {
        this.setYaw(0);
    }


}
