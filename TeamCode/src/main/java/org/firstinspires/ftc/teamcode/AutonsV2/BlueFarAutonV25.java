package org.firstinspires.ftc.teamcode.AutonsV2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

// Start blue storage side

@Autonomous
public class BlueFarAutonV25 extends LinearOpMode {
    public static final double dPower = 0.3;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW2Attachments r = new CompBotW2Attachments();

    @Override
    public void runOpMode() {
        r.init(hardwareMap);
        telemetry.addLine("init finished");
        telemetry.update();

        r.setBucket(1);

        waitForStart();

        runtime.reset();

        double heading = r.imu.getHeading();

        r.AEncDrive(20,55,0.15,dPower,2000);

        r.AEncDrive(-12,0,-dPower,0,2000);

        r.spin(2500);

        r.AEncDrive(12,0,dPower,0,1500);

        r.AEncDrive(-8,0,-0.15,0,2000);

        r.AEncDrive(0,-63,0,-dPower,5000);

        r.gyroTurnAbsolute(heading,0.1,2000);

        r.AEncDrive(3,0,dPower,0,2000);

        r.autonLift(new boolean[]{false,true,false},0.2);

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