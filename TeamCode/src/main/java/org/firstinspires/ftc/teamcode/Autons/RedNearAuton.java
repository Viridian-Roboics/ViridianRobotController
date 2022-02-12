package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.driveUntilMechStop;
import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.nEncDrive;
import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;
import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous(name="Red Warehouse Side")
public class RedNearAuton extends LinearOpMode {
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
            pos = r.p.getPositions();
        }

        r.phoneCam.stopStreaming();

        runtime.reset();

        r.setBucket(1);

        // line up with drop
        r.AEncDrive(0,-32,0,-dPower,3000);

        r.AEncDrive(-3,0,-0.15,0,1500);

        r.AEncDrive(20,0,dPower,0,2000);

        //lift and drop
        r.autonLift(pos,dPower);
        telemetry.addLine("finished with lift");
        telemetry.update();

        r.AEncDrive(-22,0,dPower,0,2000);


        // Strafe to warehouse
        r.gyroTurn(90,0.2,2000);
        telemetry.addLine("finished with turn");
        telemetry.update();
        r.AEncDrive(0,-15,0,-0.15,1000); // bang into wall
        r.AEncDrive(-72,-6,-1,-0.1, 3000);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}