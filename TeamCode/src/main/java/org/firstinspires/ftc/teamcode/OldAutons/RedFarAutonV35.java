package org.firstinspires.ftc.teamcode.OldAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

// Start blue storage side

@Autonomous(name="Red Carousel Side",group="Old Autons")
public class RedFarAutonV35 extends LinearOpMode {
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
        r.restBucket();


        runtime.reset();

        double heading = r.imu.getHeading();

        r.AEncDrive(15,-43,dPower/2,-dPower,2500, telemetry);

        r.AEncDrive(-14,0,-0.15,0,1000);

        r.spinReverse(2000);

        r.AEncDrive(4,0,0.15,0,600);

        r.AEncDrive(0,-4,0,-0.15,500);

        r.gyroTurnAbsolute(heading,0.1,2000);

        r.AEncDrive(0,64,0,dPower-.1,4500);

        r.AEncDrive(7,0,dPower-.4,0,3000);

        r.autonLift(pos);

        r.AEncDrive(-22,0,-dPower,0,2000);

        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDrive(28,-63,0.3,-0.8,5000);



        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}