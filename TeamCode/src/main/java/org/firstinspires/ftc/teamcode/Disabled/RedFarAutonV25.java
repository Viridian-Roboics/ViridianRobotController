package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous(name="Red Carousel Sideaaaaaaa")
@Disabled
public class RedFarAutonV25 extends LinearOpMode {
    public static final double dPower = 0.6;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW2Attachments r = new CompBotW2Attachments();

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true, telemetry,"red");
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

        r.AEncDrive(20,-55,0.15,-dPower,2000);

        r.AEncDrive(-12,0,-dPower,0,1500);

        r.spinReverse(2000);

        r.AEncDrive(4,0,dPower,0,1500);

        r.AEncDrive(0,60,0,dPower,5000);

        r.gyroTurnAbsolute(heading,0.1,1500);

        r.AEncDrive(6,0,dPower,0,2000);

        r.autonLift(pos,0.2);

        r.AEncDrive(-22,0,-dPower,0,2500);

        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDrive(32,-65.5,0.3,-0.8,5000);



        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}