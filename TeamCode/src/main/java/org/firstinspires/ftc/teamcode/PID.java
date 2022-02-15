package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double[] coeffs;
    public double refValue, intError = 0, prevError = 0;
    private ElapsedTime e;
    private boolean first = true;

    public PID(double[] coeffs, double refValue) {
        this.coeffs = coeffs;
        this.refValue = refValue;
        e = new ElapsedTime();
    }
    public void setRefVal(double rv) {
        refValue = rv;
    }

    public double calculate(double newValue) {
        double error = newValue - refValue;
        if(first) {
            first = false;
            return error*coeffs[0];
        }
        double dt = e.milliseconds();
        e.reset();
        double dervError = (error-prevError)/dt;
        intError += error*dt;
        prevError = error;
        return error*coeffs[0] + dervError*coeffs[2] + intError*coeffs[1];
    }
}
