package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="F0 Drive Debugger")

public class F0teleop extends OpMode {
    F0Hardware f = new F0Hardware();
    @Override
    public void init() {
        f.init(hardwareMap);
    }

    @Override
    public void loop() {
        double accel = 0.25*(gamepad1.right_trigger - gamepad1.left_trigger);
        double turn  = accel*gamepad1.left_stick_x;

        f.right.setPower(accel - turn);
        f.left.setPower(accel + turn);
    }

    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
