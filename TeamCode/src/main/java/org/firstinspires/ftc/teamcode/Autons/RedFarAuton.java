package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.driveUntilMechStop;
import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.nEncDrive;
import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous(name="Red Carousel Side")
public class RedFarAuton extends LinearOpMode {
    public static final double dPower = 0.3;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW1Attachments r = new CompBotW1Attachments();

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true, telemetry,"red");
        telemetry.addLine("init finished");
        telemetry.update();
        boolean[] pos = {false,false,false};
        ElapsedTime e = new ElapsedTime();
        while(!isStarted()) {
            pos = r.q.getPositions(); // Detection
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
        r.spin2(2000);

        // Move over to Hub
        r.AEncDrive(0,52,0,dPower,4000);

        // Align with initial heading
        r.gyroTurnAbsolute(heading,0.2,2000);

        //lift and drop
        r.autonLift(pos,dPower);
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