package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous(name="Red Carousel Side New")
public class RedFarAutonNew extends LinearOpMode {
    public static final double dPower = 0.3;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW1Attachments r = new CompBotW1Attachments();

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
        r.setBucket(1);

        runtime.reset();

        double heading = r.imu.getHeading();

        // Strafe over to carousel
        r.AEncDrive(0,-30,0,-0.2,4000);

        // Align with initial heading
        r.gyroTurnAbsolute(heading,0.2,2000);

        // Spin the duck
        r.spin(2000);

        // Move over to Hub
        r.AEncDrive(0,50,0,dPower);

        // Align with initial heading
        r.gyroTurnAbsolute(heading,0.2,2000);

        r.AEncDrive(-5,0,0,-dPower);
        r.AEncDrive(18,0,0,dPower);

        //lift and drop
        r.fixBucket();
        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
            r.lowLift();
            r.setBucket(0.25);
        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
            r.medLift();
            r.setBucket(0.25);
        } else {// right
            r.highLift();
            r.setBucket(0.25);
        }
        r.AEncDrive(-20,0,-dPower,0);
        sleep(1000);
        r.fixBucket();
        r.setBucket(1);
        r.zeroLift();
        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDrive(30.5,-55.5,0.3,-0.8,5000);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}