package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;
import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;

import java.util.Arrays;

@Autonomous
public class Blue_Warehouse_Side_V3 extends LinearOpMode {
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
            pos = r.p.getPositions();
        }

        r.phoneCam.stopStreaming();

        r.restBucket();

        runtime.reset();

        // line up with drop
        r.AEncDrive(0,24.5,dPower-.2,4000);

        r.AEncDrive(-3,0,0.15,1500);

        r.AEncDrive(22.5,0,0.3,3500);

        //lift and drop
        telemetry.addLine(Arrays.toString(pos));
        telemetry.update();
        r.autonLift(pos);
        telemetry.addLine("finished with lift");
        telemetry.update();

        r.AEncDrive(-22,0,dPower,2500);

        // Strafe to warehouse
        r.gyroTurn(270,0.2,2000);
        telemetry.addLine("finished with turn");
        telemetry.update();
        r.AEncDrive(0,9,0.15,1000); // bang into wall
        r.AEncDrive(-72,6,1, 3000);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}