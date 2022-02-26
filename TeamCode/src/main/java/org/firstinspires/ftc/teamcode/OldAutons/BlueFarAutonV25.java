package org.firstinspires.ftc.teamcode.OldAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

// Start blue storage side

@Autonomous(name="Blue Carousel Side",group="Old Autons")
public class BlueFarAutonV25 extends LinearOpMode {
    public static final double dPower = 1;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW2Attachments r = new CompBotW2Attachments();

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

        double sqrt = Math.sqrt(Math.pow(20, 2) + Math.pow(35, 2));
        r.AEncDrive(20,35,20* sqrt *dPower,35*sqrt *dPower,2500, telemetry);

        r.AEncDrive(-16,0,-0.15,0,2000);

        r.spin(2000);

        ElapsedTime x = new ElapsedTime();

        r.AEncDrive(4,0,0.2,0,1000);

        while(x.milliseconds() < 500) {
            r.drive(0.2,0,0);
        }

        r.AEncDrive(0,-61,0,-0.5,5000);

        r.gyroTurnAbsolute(heading,0.2,2000);

        r.AEncDrive(7,0,0.4,0,3500);


        r.autonLift(pos);

        r.AEncDrive(-22,0,-dPower,0,2000);

        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        sqrt = Math.sqrt(Math.pow(30, 2) + Math.pow(55.5, 2));
        r.AEncDrive(30,55.5,30/sqrt*dPower,55.5/sqrt*dPower,7000);



        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}