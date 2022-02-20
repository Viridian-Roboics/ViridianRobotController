package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

@TeleOp
public class arush_mode extends OpMode {
    CompBotW2Attachments r = new CompBotW2Attachments();
    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper) {
            r.setBucket(0);
        } else {
            r.setBucket(1);
        }
    }
}
