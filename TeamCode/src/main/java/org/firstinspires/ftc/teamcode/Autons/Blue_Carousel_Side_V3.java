package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;
import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;

@Autonomous
public class Blue_Carousel_Side_V3 extends LinearOpMode {
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

        r.AEncDrive(15,35,dPower,2500);

        r.AEncDrive(-12,0,0.15,1000);

        r.spin(2000);

        r.AEncDrive(4,0,0.15,600);

        r.AEncDrive(0,4,0.2,1000);

        r.gyroTurnAbsolute(heading,0.1,2000);

        r.AEncDrive(0,-61,dPower-.15,4500);

        r.AEncDrive(13,0,0.2,3500);

        r.autonLift(pos);

        r.AEncDrive(-22,0,dPower,2000);

        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDrive(28,55.5,0.8,5000);



        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}