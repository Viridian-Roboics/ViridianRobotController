package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="F0teleopSteeringDriftModeRampedAccel")

public class F0teleopSteeringDriftModeRampedAccel extends OpMode {
    F0HardwareSteeringWithGyro f = new F0HardwareSteeringWithGyro();

    double maxP = 1;
    double turnCorrK = 0.03;
    double accel;

    boolean gyroReset = false, ramping = false;

    ElapsedTime t = new ElapsedTime();

    @Override
    public void init() {
        f.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Get max power
        if(gamepad1.dpad_left && maxP > 0.05) {
            maxP -= 0.01;
        } else if(gamepad1.dpad_right && maxP < 1) {
            maxP += 0.01;
        }
        telemetry.addData("maxP", maxP);

        double newAccel = maxP*(gamepad1.right_trigger - gamepad1.left_trigger);
        double turn  = gamepad1.left_stick_x;
        int turnTrim = 0;

        if (gamepad1.x)
            turnTrim++;

        if (gamepad1.y)
            turnTrim--;

        if(newAccel - accel > 0.05) {
            if(!ramping) {
                ramping = true;
                t.reset();
            }
            if(t.milliseconds() > 2) {
                accel += 0.02;
            }

        } else {
            accel = newAccel;
            if(ramping) {
                ramping = false;
            }
        }

        double rp, lp;

        if(turn != 0) {
            gyroReset = false;
            rp = accel*(1 - 0.6*turn);
            lp = accel*(1 + 0.6*turn);
        } else {
            if(!gyroReset) {
                f.imu.reset();
                gyroReset = true;
                rp = accel*(1 - 0.6*turn);
                lp = accel*(1 + 0.6*turn);
            } else {
                double corr = f.imu.getHeading()*turnCorrK;
                rp = accel*(1 - 0.6*turn + corr);
                lp = accel*(1 + 0.6*turn - corr);
            }
        }


        telemetry.addData("lp", lp);
        telemetry.addData("rp", rp);

        f.right.setPower(MathUtils.clamp(lp,-1,1));
        f.left.setPower(MathUtils.clamp(rp,-1,1));

        // Steering
        InterpLUT servoPosition = new InterpLUT();
        servoPosition.add(-1.01,-2);
        servoPosition.add(0,0);
        servoPosition.add(1.01, 2);
        servoPosition.createLUT();
        InterpLUT speedCorrection = new InterpLUT();
        speedCorrection.add(-0.01,1);
        speedCorrection.add(1.01, 0.5);
        speedCorrection.createLUT();
        double tAng = servoPosition.get(turn)*speedCorrection.get(Math.abs(accel));
        f.steer.turnToAngle(tAng+(double)turnTrim);
        telemetry.addData("Servo angle", tAng);
        telemetry.addData("Trim Value", turnTrim);
        telemetry.update();
    }

    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
