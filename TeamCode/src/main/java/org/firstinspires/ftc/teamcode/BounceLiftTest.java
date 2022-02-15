package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

@Autonomous
public class BounceLiftTest extends LinearOpMode {
    CompBotW2Attachments r = new CompBotW2Attachments();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        r.fixBucket();
        r.highLift();
        r.setBucket(0.25);
        sleep(1000);
        r.setBucket(1);
        r.zeroLift();
    }
}
