package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

// Start blue storage side

@Autonomous(name="Blue Carousel Side")
public class RedFarAutonV35 extends LinearOpMode {
    public static final double dPower = 0.6;
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
        r.setBucket(1);

        runtime.reset();

        double heading = r.imu.getHeading();

        r.AEncDrive(15,-35,dPower/2,-dPower,2500, telemetry);

        r.AEncDrive(-12,0,-0.15,0,1000);

        r.spin(2000);

        r.AEncDrive(4,0,0.15,0,600);

        r.AEncDrive(0,-4,0,-0.15,500);

        r.gyroTurnAbsolute(heading,0.1,2000);

        r.AEncDrive(0,63,0,dPower,4000);

        r.AEncDrive(7,0,dPower-.2,0,2500);

        r.autonLift(pos,0.2);

        r.AEncDrive(-22,0,-dPower,0,2000);

        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDrive(30,-55.5,0.3,-0.8,5000);



        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}