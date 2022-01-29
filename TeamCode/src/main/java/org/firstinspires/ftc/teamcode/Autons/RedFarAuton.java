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

@Autonomous(name="Red Carousel Side")
@Disabled
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
            pos = r.q.getPositions();
        }

        r.phoneCam.stopStreaming();
        r.setBucket(1);

        runtime.reset();

        // Turn
        r.AEncDrive(6,0,dPower,0);
        r.gyroTurn(90,dPower,3000);

        // Strafe over
        r.AEncDrive(14,0,dPower,0);
        r.AEncDrive(10,0,dPower,0,2000);

        // Spin
        r.spin(2000);

        // Move over
        r.AEncDrive(-56,0,-dPower,0);
        r.gyroTurn(-90,-dPower,3000);
        r.AEncDrive(-6,0,0.15,0,1500);
        
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

        // Drive to depot
        r.AEncDrive(12,-56,dPower,-dPower);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}