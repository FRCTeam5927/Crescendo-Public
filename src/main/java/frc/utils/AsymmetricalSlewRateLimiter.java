package frc.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class AsymmetricalSlewRateLimiter {
    private SlewRateLimiter _acclimiter;
    private SlewRateLimiter _decclimiter;
    private SlewRateLimiter _movelimiter;
    private SlewRateLimiter _directionchangelimiter;
    private double zDeadband;
    private double prev_val = 0.0;
    private double prev_limitedVal = 0.0;

    public AsymmetricalSlewRateLimiter(double acclimit, double decclimit, double movelimit, double dchangelimit, double z_deadband) {
        _acclimiter = new SlewRateLimiter(acclimit);
        _decclimiter = new SlewRateLimiter(decclimit);
        _movelimiter = new SlewRateLimiter(movelimit);
        _directionchangelimiter = new SlewRateLimiter(dchangelimit);
        zDeadband = z_deadband;
    }

    public double calculate(double magnitude, double angle, boolean changeDir) {
        boolean isRising = (magnitude-prev_val)>0.01;
        boolean isFalling = (magnitude-prev_val)<-0.01;
        prev_val = magnitude;
        double limitedInput = 0.0;


        if(isFalling){
            _movelimiter.reset(prev_limitedVal);
            _acclimiter.reset(prev_limitedVal);
            limitedInput = _decclimiter.calculate(magnitude);
        } else if(isRising){
            _movelimiter.reset(prev_limitedVal);
            _decclimiter.reset(prev_limitedVal);
            limitedInput = _acclimiter.calculate(magnitude);
        } else  {
//            _acclimiter.reset(prev_limitedVal);
//            _decclimiter.reset(prev_limitedVal);
//            limitedInput = _movelimiter.calculate(magnitude);
        }

        prev_limitedVal = limitedInput;
        return limitedInput;

//        return magnitude;
    }

    public void zeroAll() {
        _acclimiter.reset(0.0);
        _decclimiter.reset(0.0);
    }


}
