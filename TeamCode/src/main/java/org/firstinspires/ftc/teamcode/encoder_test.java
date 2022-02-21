package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;

@TeleOp
public class encoder_test extends LinearOpMode {
    CompBotW3Attachments r = new CompBotW3Attachments();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        r.AEncDriveLinearSlow(30, 0, 0.5, 2000);
        r.AEncDriveLinearSlow(-30,25, 0.5, 2000);
        r.AEncDriveLinearSlow(5,5,0.2,2000);
        sleep(1000);
    }
}
