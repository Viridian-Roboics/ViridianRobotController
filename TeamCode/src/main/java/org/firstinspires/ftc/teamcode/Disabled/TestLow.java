package org.firstinspires.ftc.teamcode.Disabled;

import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.driveUntilMechStop;
import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.nEncDrive;
import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.runMotorTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

@Autonomous(name="TestLow")
@Disabled
public class TestLow extends LinearOpMode {

    CompBotW1Attachments r = new CompBotW1Attachments();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        r.init(hardwareMap);
        telemetry.addLine("init finished");
        telemetry.update();
        waitForStart();

        if (r.getLiftPos() > 5000) { //overflow
            int dif = (r.getLiftPos() - 5000);
            r.moveLiftPosition(dif, 1);
        } else if (r.getLiftPos() < 5000) {
            int dif = -1 * (r.getLiftPos() - 5000);
            r.moveLiftPosition(dif, -1);
        }
        sleep(1000);
        r.setBucket(.5);
        sleep(1000);
        r.moveLiftPosition(-1700, -1);
        sleep(1000);
        r.setBucket(.2);
        sleep(1000);
        r.moveLiftPosition(1700, 1);
        sleep(1000);
        r.setBucket(1);
        sleep(1000);
        r.moveLiftPosition(-5000, -1);
        sleep(1000);
        telemetry.addLine("program done");
        telemetry.update();
    }
}

