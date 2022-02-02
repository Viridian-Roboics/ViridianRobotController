package org.firstinspires.ftc.teamcode.FiniteState;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double[] coeff = new double[3];
    double initial, current = 0, pastError = 0, intError = 0;
    ElapsedTime e = new ElapsedTime();
    public PID(double[] coeff, double initial) {
        e.reset();
        this.coeff = coeff;
        this.initial = initial;
    }
    public double calculate(double current) {
        double eTime = e.milliseconds();
        e.reset();
        double error = current - initial;
        double dervError = (error - pastError)/eTime;
        intError += error*eTime;
        double totalError = coeff[0]*error + coeff[1]*intError + coeff[2]*dervError;
        pastError = error;
        return totalError;
    }
}
