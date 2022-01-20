package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

@TeleOp(name="liftTestActual")
public class liftTestActual extends OpMode {
    CompBotW1Attachments r = new CompBotW1Attachments();
    double posInit = 0;

    @Override
    public void init() {
        r.init(hardwareMap);
        posInit = r.lift.getCurrentPosition();
    }

    @Override
    public void loop() {
        double p;
        if(gamepad1.dpad_up) {
            p = 1;
        }
        else if(gamepad1.dpad_down) {
            p = -1;
        } else {
            p = 0;
        }
        if(gamepad1.a) {
            r.setLiftPower(p);
        } else {
            r.safeLiftDrive(p);
        }
        if(gamepad1.left_bumper){
            r.setBucket(.3);
        }
        else if(gamepad1.right_bumper){
            r.setBucket(1);
        }
        telemetry.addData("Lift position",r.lift.getCurrentPosition()-posInit);
        telemetry.addLine(String.valueOf(Boolean.valueOf((r.lift.getCurrentPosition() > r.liftMax && Math.signum(p) == 1) || (r.lift.getCurrentPosition() < r.liftZero && Math.signum(p) == -1))));
        telemetry.update();
    }
}
