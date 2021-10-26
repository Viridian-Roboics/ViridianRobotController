package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

@TeleOp(name="F0 Drive Software V2.0")

public class F0teleopSteeringDriftModeRampedAccel extends OpMode {
    F0HardwareSteeringWithGyro f = new F0HardwareSteeringWithGyro();

    double maxP = 1;
    double turnCorrK = 0.03;
    double accel;

    double turnTrim = 0;

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
            maxP -= 0.002;
        } else if(gamepad1.dpad_right && maxP < 1) {
            maxP += 0.002;
        }
        telemetry.addData("maxP", maxP);

        double accel = maxP*(gamepad1.right_trigger - gamepad1.left_trigger);
        double turn  = gamepad1.left_stick_x;


        double rp, lp;
        /*
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
        */
        rp = accel*(1 - 0.6*turn);
        lp = accel*(1 + 0.6*turn);

        telemetry.addData("lp", lp);
        telemetry.addData("rp", rp);

        f.right.setPower(MathUtils.clamp(lp,-1,1));
        f.left.setPower(MathUtils.clamp(rp,-1,1));

        // turntrim
        if(gamepad1.x) {
            turnTrim += 0.002;
        } else if(gamepad1.y) {
            turnTrim -= 0.002;
        }

        // Steering
        InterpLUT servoPosition = new InterpLUT();
        servoPosition.add(-1.01,-1);
        servoPosition.add(0,0);
        servoPosition.add(1.01, 1.5);
        servoPosition.createLUT();
        InterpLUT speedCorrection = new InterpLUT();
        speedCorrection.add(-0.01,1);
        speedCorrection.add(1.01, 0.2);
        speedCorrection.createLUT();
        double tAng = servoPosition.get(turn)*speedCorrection.get(Math.abs(accel));
        f.steer.turnToAngle(tAng+turnTrim);
        telemetry.addData("Servo angle", tAng);
        telemetry.addData("Trim Value", turnTrim);
        telemetry.update();
    }

    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
