package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

@TeleOp
@Disabled
public class TeleopServo extends OpMode {
    CompBotW2Attachments r = new CompBotW2Attachments();
    ElapsedTime x = new ElapsedTime();
    ElapsedTime a = new ElapsedTime();

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.x) {
            r.spin0.setPower(0.25);
            r.spin1.setPower(0.25);
        } else {
            x.reset();
        }
        if(gamepad1.a) {
            r.spin0.setPower(1);
            r.spin1.setPower(1);
        } else {
            a.reset();
        }
        telemetry.addData("Time since x was last pressed",x.milliseconds());
        telemetry.addData("Time since a was last pressed",a.milliseconds());
        telemetry.update();
    }
}
