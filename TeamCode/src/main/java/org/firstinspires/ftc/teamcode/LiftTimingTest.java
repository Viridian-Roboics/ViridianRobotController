package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;

@Autonomous
@Disabled
public class LiftTimingTest extends LinearOpMode {
    CompBotW3Attachments r = new CompBotW3Attachments();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        telemetry.setAutoClear(false);
        ElapsedTime e = new ElapsedTime();
        r.lowLift();
        telemetry.addLine("Zero to low");
        telemetry.addLine(String.valueOf(e.milliseconds()));
        sleep(500);
        r.zeroLift();
        sleep(500);
        e.reset();
        r.medLift();
        telemetry.addLine("Zero to med");
        telemetry.addLine(String.valueOf(e.milliseconds()));
        sleep(500);
        r.zeroLift();
        sleep(500);
        e.reset();
        r.highLift();
        telemetry.addLine("Zero to high");
        telemetry.addLine(String.valueOf(e.milliseconds()));
        sleep(500);
        r.zeroLift();
        sleep(500);
        e.reset();
        r.setLiftPosition(r.liftSafe, 1);
        telemetry.addLine("Zero to safe");
        telemetry.addLine(String.valueOf(e.milliseconds()));
        sleep(500);
        e.reset();
        r.lowLift();
        telemetry.addLine("Safe to low");
        telemetry.addLine(String.valueOf(e.milliseconds()));
        sleep(500);
        r.setLiftPosition(r.liftSafe, 1);
        sleep(500);
        e.reset();
        r.medLift();
        telemetry.addLine("Safe to med");
        telemetry.addLine(String.valueOf(e.milliseconds()));
        sleep(500);
        r.setLiftPosition(r.liftSafe, 1);
        sleep(500);
        e.reset();
        r.highLift();
        telemetry.addLine("Safe to high");
        telemetry.addLine(String.valueOf(e.milliseconds()));
        sleep(500);
        r.setLiftPosition(r.liftSafe, 1);
        sleep(500);
        r.zeroLift();
        telemetry.update();
        while(opModeIsActive()) {

        }
    }
}
