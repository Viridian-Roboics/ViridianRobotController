package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.driveUntilMechStop;
import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.nEncDrive;
import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous(name="Red Warehouse Side")
@Disabled
public class RedNearAuton extends LinearOpMode {
    public static final double dPower = 0.35;
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
            pos = r.q.getPositions();
        }

        r.phoneCam.stopStreaming();

        runtime.reset();

        r.setBucket(1);

        // line up with drop
        r.AEncDrive(0,-20,0,-dPower);

        //lift and drop
        r.AEncDrive(-5,0,-0.15,0);
        r.AEncDrive(20,0,dPower,0);
        r.fixBucket();
        sleep(1000);
        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
            r.lowLift();
            r.setBucket(0.25);
        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
            r.medLift();
            r.setBucket(0.25);
        } else {// right
            r.highLift();
            r.setBucket(0.25);
        }
        sleep(1000);
        r.AEncDrive(-20,0,-dPower,0);
        sleep(1000);
        r.fixBucket();
        r.setBucket(1);
        r.zeroLift();
        telemetry.addLine("finished with lift");
        telemetry.update();


        // Strafe to warehouse
        r.gyroTurn(-90,0.2);
        telemetry.addLine("finished with turn");
        telemetry.update();
        r.AEncDrive(0,10,0,dPower, 3000);
        r.AEncDrive(0,15,0,0.15,3000);
        r.AEncDrive(100,0,1,0);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}