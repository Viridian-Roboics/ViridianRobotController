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

@Autonomous(name="Blue Warehouse Side")
public class BlueNearAuton extends LinearOpMode {
    public static final double dPower = 0.35;
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
            pos = r.p.getPositions();
        }

        r.phoneCam.stopStreaming();

        runtime.reset();

        r.setBucket(1);

        // line up with drop
        r.AEncDrive(0,22,0,dPower,3000);

        //lift and drop
        r.autonLift(pos,dPower);
        telemetry.addLine("finished with lift");
        telemetry.update();


        // Strafe to warehouse
        r.gyroTurn(270,0.2,3000);
        telemetry.addLine("finished with turn");
        telemetry.update();
        r.AEncDrive(0,15,0,0.15,2000); // bang into wall
        r.AEncDrive(0,-1.5,0,-0.15,2000); // bang into wall
        r.AEncDrive(-60,0,-1,0);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}