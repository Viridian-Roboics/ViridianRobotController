package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

@Autonomous
@Disabled
public class LiftTest2 extends LinearOpMode {
    CompBotW1Attachments r = new CompBotW1Attachments();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        r.fixBucket();
        r.lowLift();
        telemetry();
        r.setBucket(0.25);
        telemetry();
        sleep(1000);
        r.fixBucket();
        r.setBucket(1);
        r.zeroLift();

        // Drive 20 inches forward for high and middle
        // Drive 16 inches forward for low
    }
    public void telemetry() {
        telemetry.addData("bucketpos",r.bucket0.getPosition());
        telemetry.addData("bucket2pos",r.bucket1.getPosition());
        telemetry.addData("liftPos",r.getLiftPos());
        telemetry.update();
    }
}
