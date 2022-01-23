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

@Autonomous(name="Blue Carousel Side")
public class BlueFarAuton extends LinearOpMode {
    public static final double dPower = 0.3;
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
        r.setBucket(1);

        runtime.reset();

        double heading = r.imu.getHeading();

        // Strafe over
        r.AEncDrive(0,22,0,0.2,4000);

        r.gyroTurnAbsolute(heading,0.2,2000);

        // Spin
        r.spin(2000);

        // Move over
        r.AEncDrive(0,-50,0,-dPower);

        r.gyroTurnAbsolute(heading,0.2,2000);

        //lift and drop
        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
            r.AEncDrive(14,0,dPower,0);
            r.lowLift();
            //r.setBucket(0);
            sleep(2000);
            //r.setBucket(1);
            r.zeroLift();
            r.AEncDrive(-14,0,-dPower,0);
        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
            r.AEncDrive(6,0,dPower,0);
            r.medLift();
            //r.setBucket(0);
            sleep(2000);
            //r.setBucket(1);
            r.zeroLift();
            r.AEncDrive(-6,0,-dPower,0);
        } else {// right
            r.highLift();
            //r.setBucket(0);
            sleep(2000);
            //r.setBucket(1);
            r.zeroLift();
        }
        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDrive(30.5,55.5,0.3,0.8,5000);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}