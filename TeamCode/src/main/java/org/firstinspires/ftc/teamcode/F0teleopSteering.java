package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="F0teleopSteering")

public class F0teleopSteering extends OpMode {
    F0HardwareSteering f = new F0HardwareSteering();

    double maxP = 0.5;

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

        double accel = maxP*(gamepad1.right_trigger - gamepad1.left_trigger);
        double turn  = gamepad1.left_stick_x;

        double rp = accel*(1 - 0.3*turn), lp = accel*(1 + 0.3*turn);

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
        double tAng = servoPosition.get(turn);
        f.steer.turnToAngle(tAng);
        telemetry.addData("Servo angle", tAng);

        telemetry.update();
    }

    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
