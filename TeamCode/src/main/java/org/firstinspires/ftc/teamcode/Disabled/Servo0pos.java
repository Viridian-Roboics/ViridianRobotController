package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

@TeleOp
@Disabled
public class Servo0pos extends OpMode {
    CompBotW2Attachments r = new CompBotW2Attachments();
    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < 2000) {
            r.setBucket(0.1);
        }
        e.reset();
        while(e.milliseconds() < 2000) {
            r.setBucket(0.7);
        }
        e.reset();
    }
}
