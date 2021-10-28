package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

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
    boolean wasPressedUp = true;
    boolean wasPressedDown = true;

    double turnCorrK = 0.02; //turn correction magnitude: 0.01 - 0.04
    double gyroDelay = 250; //delay before veer correction (ms)
    double brakePower = -0.01;
    double accel;

    double turnTrim = 0;
    double lastHeading = 0;

    boolean gyroReset = false, ramping = false, gyroWait = false;

    ElapsedTime gyroRT = new ElapsedTime();

    ElapsedTime t = new ElapsedTime();

    @Override
    public void init() {
        f.init(hardwareMap);
    }

    @Override
    public void loop() {

        //Power Settings
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

        if (turn != 0) {
            gyroReset = false;
        } else {
            if (!gyroReset) { //checking for steering input
                if(!gyroWait) {
                    gyroWait = true;
                    gyroRT.reset();
                }
                else if(gyroRT.milliseconds() > gyroDelay) { //gyro delay for correction after steering input
                    gyroWait = false;
                    lastHeading = f.imu.getHeading();
                    gyroReset = true;
                }
            } else {
                double corr = (f.imu.getHeading() - lastHeading) * turnCorrK;
                turn = MathUtils.clamp(turn + corr, -1, 1);
            }
        }

        double rp, lp;

        //defining right and left motor powers
        rp = accel * (1 - 0.7 * turn);
        lp = accel * (1 + 0.7 * turn);

        //motor power control
        if(gamepad1.b) {
            f.left.setPower(brakePower);
            f.left.setPower(brakePower);
            telemetry.addData("Throttle Status", "BRAKE ENGAGED");
        } else {
            f.left.setPower(MathUtils.clamp(rp,-1,1));
            f.right.setPower(MathUtils.clamp(lp,-1,1));
            telemetry.addData("Throttle Percentage", Math.round(lp*100));
        }

        if (!(gamepad1.dpad_up == wasPressedUp) && gamepad1.dpad_up)
            turnCorrK =+ 0.001;
        else if (!(gamepad1.dpad_down == wasPressedDown) && gamepad1.dpad_down)
            turnCorrK =- 0.001;
        if (turnCorrK < 0)
            turnCorrK = 0;

        telemetry.addData("Correction Factor (Do not exceed 0.04)", (Math.round(turnCorrK*1000))/10);

        //steering trim
        if(gamepad1.x) {
            turnTrim += 0.002;
        } else if(gamepad1.y) {
            turnTrim -= 0.002;
        }

        //Steering
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
        telemetry.update();
    }

    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
