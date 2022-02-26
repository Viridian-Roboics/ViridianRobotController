package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;

import java.util.Arrays;

@Autonomous(name="Blue Carousel Side New",group="New Autons")
public class Blue_Carousel extends LinearOpMode {
    public static final double dPower = 1;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW3Attachments r = new CompBotW3Attachments();

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true, telemetry,"blue");
        telemetry.addLine("init finished");
        telemetry.update();
        boolean[] pos = {true,false,false};
        ElapsedTime e = new ElapsedTime();

        /*
        while(!isStarted()) {
            pos = r.p.getPositions(); // Detection
        }

         */

        waitForStart();

        r.phoneCam.stopStreaming();

        r.restBucket();

        runtime.reset();

        double heading = r.imu.getHeading();

        r.lift.setPower(1);

        r.AEncDrive(20,30,dPower,2500); // Strafe over to wall

        r.lift.setPower(0);

        ElapsedTime x = new ElapsedTime();

        r.drive(0.2,0,0);

        r.highLift();

        while(x.milliseconds() < 500) {
            r.drive(0.2,0,0);
        }

        r.AEncDrive(-12,0,0.15,3000); // Back into spinner

        r.tempBucket();

        ElapsedTime spinTimer = new ElapsedTime();
        //r.setSpin(-1);

        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
            r.lowLift();
        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
            r.medLift();
        } else {// right
            r.highLift();
        }

        while(spinTimer.milliseconds() < 2000) {
            //r.setSpin(-1);
        }

       // r.setSpin(0);

        r.AEncDrive(4,0,0.15,600);

        x.reset();
        while(x.milliseconds() < 500) {
            r.drive(0.2,0,0);
        }
        r.stopDrive();

        r.AEncDriveLinearSlow(0,-61,dPower,4500);

        r.gyroTurnAbsolute(heading,0.2,1000);

        r.AEncDriveLinearSlow(13,0,0.2,3500);

        r.topDumpBucket();

        sleep(500);

        if(r.lift.getCurrentPosition() < 3000) {
            r.setLiftPower(1);
        }

        r.AEncDriveLinearSlow(-20,0,dPower,2000);

        r.lift.setPower(0);

        x.reset();
        while(x.milliseconds() < 500) {
            r.drive(0,-0.2,0);
        }

        r.restBucket();

        telemetry.addLine("finished with lift");
        telemetry.update();

        // Drive to depot
        r.AEncDriveLinearSlow(28,55.5,0.8,5000);

        r.lift.setPower(-1);

        r.zeroLift();

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}