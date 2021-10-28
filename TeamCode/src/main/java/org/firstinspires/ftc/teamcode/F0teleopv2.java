package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="F0 Drive Software V2.1")

public class F0teleopv2 extends OpMode {
    F0HardwareSteeringWithGyro f = new F0HardwareSteeringWithGyro();

    double[] maxP = new double[] {.125, .25, .5, .75, .9};
    int powerIndex = 0;
    String[] powerSetting = new String[] {"Cruise", "Normal", "Sport", "Super Sport", "Overdrive"};

    boolean wasPressedLeft = true;
    boolean wasPressedRight = true;

    double turnCorrK = 0.0175;
    double accel;

    double turnTrim = 0;
    double lastHeading = 0;

    boolean gyroReset = false, ramping = false;

    ElapsedTime t = new ElapsedTime();

    @Override
    public void init() {
        f.init(hardwareMap);
    }

    @Override
    public void loop() {

        // Get max power

        if (!(gamepad1.dpad_left == wasPressedLeft) && gamepad1.dpad_left && maxP[powerIndex] > 0.05) {
            powerIndex--;
        } else if (!(gamepad1.dpad_right == wasPressedRight) && gamepad1.dpad_right && maxP[powerIndex] < 1) {
            powerIndex++;
        }
        if ((powerIndex > 4)) {
            powerIndex = 4;
        } else if (powerIndex < 0) {
            powerIndex = 0;
        }

        wasPressedLeft = gamepad1.dpad_left;
        wasPressedRight = gamepad1.dpad_right;

        telemetry.addData("F0 Drive Console Version 2.1 Status", "OK");
        telemetry.addData("Power Setting", powerSetting[powerIndex]);

        double accel = maxP[powerIndex] * (gamepad1.right_trigger - gamepad1.left_trigger);
        double turn = gamepad1.left_stick_x;

        double rp, lp;
        if (turn != 0) {
            gyroReset = false;
        } else {
            if (!gyroReset) {
                lastHeading = f.imu.getHeading();
                gyroReset = true;
            } else {
                double corr = (f.imu.getHeading() - lastHeading) * turnCorrK;
                turn = MathUtils.clamp(turn + corr, -1, 1);
            }
        }
        /*if (f.imu.getRevIMU().getVelocity().xVeloc > 0 && gamepad1.left_trigger > 0) {
            rp = accel * (1 - 0.7 * turn);
            lp = accel * (1 + 0.7 * turn);
        } else {
            rp = -0.05;
            lp = -0.05;
        }*/

        rp = accel * (1 - 0.7 * turn);
        lp = accel * (1 + 0.7 * turn);

        if(gamepad1.b) {
            f.left.setPower(-0.05);
            f.left.setPower(-0.05);
            telemetry.addData("Throttle Status", "BRAKE ENGAGED");
        } else {
            f.left.setPower(MathUtils.clamp(rp,-1,1));
            f.right.setPower(MathUtils.clamp(lp,-1,1));
            telemetry.addData("Throttle Percentage", Math.round(lp*100));
        }


        //trim
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
        f.steer.turnToAngle((tAng+turnTrim));
        /*telemetry.addData("Servo angle", Math.round(tAng*100)/100);
        telemetry.addData("Trim Value", Math.round(turnTrim*100)/100);*/
        telemetry.update();


    }

    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
