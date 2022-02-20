package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;

@TeleOp
public class azal0pos extends LinearOpMode {
    CompBotW3Attachments r = new CompBotW3Attachments();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        r.azalServo.setPosition(0);
        sleep(1000);
    }
}
