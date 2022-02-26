package org.firstinspires.ftc.teamcode.OldAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous(name="Blue Warehouse Side",group="Old Autons")
public class BlueNearAutonV2 extends LinearOpMode {
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
            pos = r.p.getPositions();
        }

        r.phoneCam.stopStreaming();

        runtime.reset();

        r.restBucket();


        // line up with drop
        r.AEncDrive(0,24.5,0,dPower-.2,4000);

        r.AEncDrive(-3,0,-0.15,0,1500);

        r.AEncDrive(22.5,0,dPower-.25,0,3500);

        //lift and drop
        telemetry.addLine(Arrays.toString(pos));
        telemetry.update();
        r.autonLift(pos);
        telemetry.addLine("finished with lift");
        telemetry.update();

        r.AEncDrive(-22,0,dPower,0,2500);

        // Strafe to warehouse
        r.gyroTurn(270,0.2,2000);
        telemetry.addLine("finished with turn");
        telemetry.update();
        r.AEncDrive(0,9,0,0.15,1000); // bang into wall
        r.AEncDrive(-72,6,-1,0.1, 3000);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}