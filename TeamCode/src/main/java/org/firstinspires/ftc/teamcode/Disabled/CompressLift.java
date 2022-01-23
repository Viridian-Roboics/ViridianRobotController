package org.firstinspires.ftc.teamcode.Disabled;

import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.driveUntilMechStop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3Attachments;

@TeleOp
@Disabled
public class CompressLift extends LinearOpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        driveUntilMechStop(r.lift,-1,1000);
    }
}
