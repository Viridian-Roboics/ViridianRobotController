package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;

import java.util.Arrays;

@Autonomous
public class Blue_Carousel_Side_V3_ExampleOptimized extends LinearOpMode {
    public static final double dPower = 0.6;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW3Attachments r = new CompBotW3Attachments();

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true, telemetry,"blue");
        telemetry.addLine("init finished");
        telemetry.update();
        boolean[] pos = {false,false,false};
        ElapsedTime e = new ElapsedTime();
        while(!isStarted()) {
            pos = r.p.getPositions(); // Detection
        }

        r.phoneCam.stopStreaming();

        r.restBucket();

        runtime.reset();

        double heading = r.imu.getHeading();

        r.lift.setPower(1);

        r.AEncDrive(15,35,dPower,2500); // Strafe over to wall

        r.highLift();

        r.AEncDrive(-12,0,0.15,1000); // Back into spinner

        r.tempBucket();

        ElapsedTime spinTimer = new ElapsedTime();
        r.setSpin(1);

        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
            r.lowLift();
        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
            r.medLift();
        } else {// right
            r.highLift();
        }

        while(spinTimer.milliseconds() < 2000) {
            r.setSpin(1);
        }

        r.setSpin(0);

        r.AEncDrive(4,0,0.15,600);

        r.AEncDrive(0,4,0.2,1000);

        r.gyroTurnAbsolute(heading,0.1,2000);

        r.AEncDrive(0,-61,dPower-.15,4500);

        r.AEncDrive(13,0,0.2,3500);

        r.topDumpBucket();

        sleep(500);

        r.highLift();

        r.restBucket();

        r.lift.setPower(-1);

        r.AEncDrive(-22,0,dPower,2000);

        r.lift.setPower(0);

        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDrive(28,55.5,0.8,5000);

        r.zeroLift();

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}