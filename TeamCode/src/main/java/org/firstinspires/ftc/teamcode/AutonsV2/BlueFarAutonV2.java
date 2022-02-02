package org.firstinspires.ftc.teamcode.AutonsV2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

// Start blue storage side

@Autonomous(name="Blue Carousel Side V2")
public class BlueFarAutonV2 extends LinearOpMode {
    public static final double dPower = 0.3;
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

        // Strafe over to carousel
        r.AEncDrive(18,21,0.15,0.2,3000);

        r.AEncDrive(-12,0,0.2,0,3000);

        // Align with initial heading
        r.gyroTurnAbsolute(heading,0.2,1000);

        // Spin the duck
        r.spin(1000);
        sleep(1000);

        // Move over to Hub
        r.AEncDrive(12,0,0.5,1000);
        r.AEncDrive(-18,-52,-0.1,-dPower,4000);

        // Align with initial heading
        r.gyroTurnAbsolute(heading,0.2,2000);

        //lift and drop
        r.autonLift(pos,dPower);
        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDrive(32,55.5,0.3,0.8,5000);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}