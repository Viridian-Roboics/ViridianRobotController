package org.firstinspires.ftc.teamcode.AutonRecorder;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;
import org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class AutonPlayer extends LinearOpMode {
    ArrayList<Event> events = new ArrayList<>();
    CompBotW1Attachments r = new CompBotW1Attachments();
    ElapsedTime e = new ElapsedTime();
    boolean reset = true;
    double initialHeading, error;
    boolean headingReset = false;
    double liftPower = 1;
    Scanner sc;

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        // Read file
        try {
            sc = new Scanner(new File("/sdcard/path/to/your/file.ext"));
        } catch (FileNotFoundException fileNotFoundException) {
            fileNotFoundException.printStackTrace();
        }

        waitForStart();
        e.reset();
        while(true) {
            if(isStopRequested()) {
                r.stop();
                break;
            }
            if(e.milliseconds() > 30) {
                // Run teleop
                Event gamepad1 = new Event(sc.nextLine());

                double y, x, turn;
                y = gamepad1.left_stick_y;
                x = -1*gamepad1.left_stick_x;
                turn = -1*gamepad1.right_stick_x;

                // Deadzone
                y = (Math.abs(y)>0.05 ? y : 0);
                x = (Math.abs(x)>0.05 ? x : 0);
                turn = (Math.abs(turn)>0.05 ? turn : 0);

                // Power adjust
                y *= (gamepad1.right_stick_button ?0.4:1);
                x *= (gamepad1.right_stick_button ?0.4:1);
                turn *= (gamepad1.right_stick_button ?0.4:1);

                if(Math.abs(y) > Math.abs(x)) {
                    x = 0;
                } else {
                    y = 0;
                }
                if (Math.abs(turn) < 0.1 && Math.abs(x) > 0 && Math.abs(y) > 0) {
                    if(!headingReset) {
                        initialHeading = r.imu.getHeading();
                        headingReset = true;
                    } else {
                        error = r.imu.getHeading() - initialHeading;
                        turn = CompBotV3.corrCoeff*error;
                    }
                } else {
                    headingReset = false;
                }
                r.fl.setPower(MathUtils.clamp(y+x+turn ,-1,1));
                r.fr.setPower(MathUtils.clamp(-(y-x-turn),-1,1));
                r.bl.setPower(MathUtils.clamp(y-x+turn,-1,1));
                r.br.setPower(MathUtils.clamp(-(y+x-turn),-1,1));

                r.intake.setPower((gamepad1.a?1:0) - (gamepad1.b?1:0));
                r.spin0.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
                r.spin1.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

                if(gamepad1.dpad_up) {
                    r.stopPoweredHold();
                    r.setLiftPower(liftPower);
                } else if(gamepad1.dpad_down) {
                    r.stopPoweredHold();
                    r.setLiftPower(-liftPower);
                } else {
                    r.poweredHoldCycle();
                }

                if(gamepad1.right_stick_button) {
                    r.imu.reset();
                }

                r.setBucketOverride(gamepad1.left_bumper?.3:1);

                if(gamepad1.y){
                    r.ShareGoal();
                }
                events.remove(0);
            }

            telemetry.addData("BucketPos: ", r.BucketPosition());
            telemetry.addData("liftPos: ", r.getLiftPos());
            telemetry.update();

            if(events.size() == 0) {
                r.stop();
                break;
            }
        }
    }
}
