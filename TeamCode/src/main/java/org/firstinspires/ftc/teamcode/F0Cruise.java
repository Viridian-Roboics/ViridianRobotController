package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="F0Cruise")
@Disabled

public class F0Cruise extends OpMode {
    F0HardwareSteering f = new F0HardwareSteering();

    double maxP = 1;
    double turnTrim = 0;

    @Override
    public void init() {
        f.init(hardwareMap);
    }

    @Override
    public void loop() {

        boolean isXPressed = false;
        boolean isYPressed = false;
        // Get max power
        if(gamepad1.dpad_left && maxP > 0.05) {
            maxP -= 0.01;
        } else if(gamepad1.dpad_right && maxP < 1) {
            maxP += 0.01;
        }
        telemetry.addData("maxP", maxP);



        double accel = maxP*(gamepad1.right_trigger - gamepad1.left_trigger);
        double turn  = gamepad1.left_stick_x;

        double rp, lp;
        if(gamepad1.a) {
            rp = 2*accel;
            lp = 2*accel;
        } else {
            rp = clamp(accel*(1 - 0.6*turn), -1, 1)/5;
            lp = clamp(accel*(1 + 0.6*turn), -1, 1)/5;
        }

        telemetry.addData("lp", lp);
        telemetry.addData("rp", rp);

        f.right.setPower(lp);
        f.left.setPower(rp);

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
        double tAng = servoPosition.get(turn)*speedCorrection.get(Math.abs(accel))/1.25;
        f.steer.turnToAngle(tAng+turnTrim);
        telemetry.addData("Servo Control Angle", tAng);
        telemetry.addData("Servo Trim", turnTrim);
        telemetry.addData("Servo Absolute Angle", tAng+turnTrim);
        telemetry.update();
    }


    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
