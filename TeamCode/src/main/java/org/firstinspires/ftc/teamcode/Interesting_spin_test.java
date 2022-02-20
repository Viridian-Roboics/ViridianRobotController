package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

@Autonomous
public class Interesting_spin_test extends LinearOpMode {
    CompBotW2Attachments r = new CompBotW2Attachments();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < 1000) {
            r.spin0.setPower(0.25);
            r.spin1.setPower(0.25);
        }
        e.reset();
        while(e.milliseconds() < 1500) {
            r.spin0.setPower(1);
            r.spin1.setPower(1);
        }
        r.spin0.setPower(0);
        r.spin1.setPower(0);
    }
}
