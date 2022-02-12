package org.firstinspires.ftc.teamcode.AutonFinite;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

import java.util.Arrays;

@Autonomous
@Disabled
public class BlueCarouselAutonConversion extends LinearOpMode {
    public static final double dPower = 0.3;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW2Attachments r = new CompBotW2Attachments();

    FSExecutor f;

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

        double heading = r.imu.getHeading();

        f = new FSExecutor(r,telemetry);

        // Instructions
        f.driveInstructions.add(String.format("linearDrive 20 55 0.15 %s 2000",dPower));
        f.driveInstructions.add(String.format("linearDrive -12 0 -%s 0 2000",dPower));
        f.driveInstructions.add("broadcast spin");
        f.driveInstructions.add("check spinFinished");
        f.driveInstructions.add(String.format("linearDrive 12 0 %s 0 1500",dPower));
        f.driveInstructions.add("linearDrive -8 0 -0.15 0 2000");
        f.driveInstructions.add(String.format("linearDrive 0 -63 0 -%s 5000",dPower));
        f.driveInstructions.add(String.format("turn absolute %s 0.1 2000",heading));
        f.driveInstructions.add(String.format("linearDrive 3 0 %s 0 2000",dPower));
        f.driveInstructions.add("send drop");
        f.driveInstructions.add("check dropFinished");
        f.driveInstructions.add("linearDrive -22 0 0.15 0 3000");
        f.driveInstructions.add("linearDrive 32 55.5 0.3 0.8 5000");

        f.spinInstructions.add("check spin");
        f.spinInstructions.add("spin forward 2500");
        f.spinInstructions.add("send spinFinished");

        f.liftInstructions.add(String.format("lift absolute %s 1",r.liftSafe));
        f.liftInstructions.add("send fix1");
        f.liftInstructions.add("check fix1done");
        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
            f.liftInstructions.add(String.format("lift absolute %s 1",r.liftZero+CompBotW2Attachments.lowLift));
        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
            f.liftInstructions.add(String.format("lift absolute %s 1",r.liftZero+CompBotW2Attachments.medLift));
        } else {// right
            f.liftInstructions.add(String.format("lift absolute %s 1",r.liftZero+CompBotW2Attachments.highLift));
        }
        f.liftInstructions.add("check dropFinished");
        f.liftInstructions.add(String.format("lift absolute %s 1",r.liftSafe));
        f.liftInstructions.add("send fix2");
        f.liftInstructions.add("check fix2done");
        f.liftInstructions.add(String.format("lift absolute %s 1",r.liftZero));

        f.bucketInstructions.add("check fix1");
        f.bucketInstructions.add("bucket 0.3 1000");
        f.bucketInstructions.add("send fix1done");
        f.bucketInstructions.add("check drop");
        f.bucketInstructions.add("bucket 1 1000");
        f.bucketInstructions.add("send dropFinished");
        f.bucketInstructions.add("check fix2");
        f.bucketInstructions.add("bucket 0 1000");
        f.bucketInstructions.add("send fix2done");

        runtime.reset();

        while(opModeIsActive() && !f.outOfInstructions) {
            f.loop();
        }

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}
