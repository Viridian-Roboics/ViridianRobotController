package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

@Autonomous
@Disabled
public class BounceLiftTest extends LinearOpMode {
    CompBotW2Attachments r = new CompBotW2Attachments();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        r.fixBucket();
        r.medLift();
        r.setBucket(0.25);
        sleep(1000);
        r.fixBucket();
        r.zeroLift();
    }
}
