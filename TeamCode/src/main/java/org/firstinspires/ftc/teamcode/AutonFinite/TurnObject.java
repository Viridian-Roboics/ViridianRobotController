package org.firstinspires.ftc.teamcode.AutonFinite;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

public class TurnObject extends DriveObject {
    private final double speed;
    private final int time;
    private final double[] coeff = {CompBotW2Attachments.corrCoeff, 0,0};

    private double expectedHeading;

    public boolean finished = false;

    ElapsedTime e = new ElapsedTime();

    public TurnObject(CompBotW2Attachments r, String AR, double angle, double speed, int time) {
        super(r);
        this.speed = speed;
        this.time = time;
        r.useEncoders();
        r.imu.reset();
        if(AR.equals("absolute")) {
            expectedHeading = angle;
        } else {
            expectedHeading = r.imu.getHeading() + angle;
        }
        while (expectedHeading > 180)  expectedHeading -= 360;
        while (expectedHeading <= -180) expectedHeading += 360;
    }
    public void turnCycle() {
        double error = -1*(r.imu.getHeading() - expectedHeading);
        double power = (Math.abs(error) > 20) ? (Math.signum(error) * speed) : ((error / 20) * speed);
        r.fl.setPower(power);
        r.bl.setPower(power);
        r.fr.setPower(power);
        r.br.setPower(power);
        if(!(Math.abs(error) > 0.01 && e.milliseconds() < time)) {
            finished = true;
        }
    }
}
