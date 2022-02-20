package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

@TeleOp
public class Servo0pos extends OpMode {
    CompBotW2Attachments r = new CompBotW2Attachments();
    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        r.setBucket(0);
    }
}
