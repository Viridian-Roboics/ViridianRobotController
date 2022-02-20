package org.firstinspires.ftc.teamcode.Teleops;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;
import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;
import org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3;

import java.util.Dictionary;
import java.util.Hashtable;

/*
// Attachment controls
1A: intake in, 1B: intake out
1X: momentary switch for Azal servo
1Up: lift up to top dump, 1Down: lift down, 1Y: lift up to bottom dump position
1LB: Top dump, 1RB: Bottom dump
1RT - 1LT: spin
1Back: reset lift zero (active during init)
// Movement controls
2LS + 2RSX: movement
2A: Sport mode, 2X: Chill mode, 2B: Precise mode
2Y: Jiggle (shakes robot back and forth)
2Dpad: Nudge controls (1 inch in any direction, 0.2 power, 500ms)
2Back: Gyro reset
// Unused controls: Both LSB, Start, Home | 1Left, 1Right, 1LS, 1RS | 2LT, 2RT, 2LB, 2RB, 2RSY, 2RSB
*/

@TeleOp
public class Viridian_Competition_Teleop extends LinearOpMode {
    CompBotW3Attachments r = new CompBotW3Attachments();
    enum DRIVE_MODES {
        PRECISE, CHILL, SPORT, NUDGING, JIGGLING
    }
    Hashtable<DRIVE_MODES, Double> modePowers = new Hashtable<DRIVE_MODES, Double>();
    DRIVE_MODES dm = DRIVE_MODES.SPORT;
    DRIVE_MODES pdm = DRIVE_MODES.SPORT;

    @Override
    public void runOpMode() {
        r.init(hardwareMap);
        // Drive mode setup
        modePowers.put(DRIVE_MODES.NUDGING,0.2);
        modePowers.put(DRIVE_MODES.PRECISE, 0.2);
        modePowers.put(DRIVE_MODES.CHILL, 0.4);
        modePowers.put(DRIVE_MODES.SPORT, 1.0);
        modePowers.put(DRIVE_MODES.JIGGLING, 1.0);

        while(!isStarted()) {
            if(gamepad1.back) {
                r.liftZero = r.lift.getCurrentPosition();
            }

            telemetry.addLine("init finished");
            telemetry.addLine("======================================");
            telemetry.addLine("||     Now with F0-racing inspired code     ||");
            telemetry.addLine("======================================");
            telemetry.addData("liftPos", r.getLiftPos());
            telemetry.update();
        }

        while(opModeIsActive()) {
            drive();
            attachments();

            telemetry.addLine(String.valueOf(gamepad1.a));
            telemetry.addData("BucketPos", r.bs);
            telemetry.addData("liftPos", r.getLiftPos());
            telemetry.addData("Drive mode", dm);
            telemetry.update();
        }
        r.stop();
    }

    double initialHeading, error;
    ElapsedTime nudgeTimer = new ElapsedTime(), jiggleTimer = new ElapsedTime();
    boolean headingReset = false, jiggleState = false;
    public void drive() {
        if(gamepad2.a) {
            dm = DRIVE_MODES.SPORT;
            pdm = DRIVE_MODES.SPORT;
        }
        if(gamepad2.b) {
            dm = DRIVE_MODES.PRECISE;
            pdm = DRIVE_MODES.PRECISE;
        }
        if(gamepad2.x) {
            dm = DRIVE_MODES.CHILL;
            pdm = DRIVE_MODES.CHILL;
        }
        double y, x, turn;
        // Jiggle
        if(gamepad2.y) {
            if(dm != DRIVE_MODES.JIGGLING) {
                dm = DRIVE_MODES.JIGGLING;
                jiggleState = false;
                jiggleTimer.reset();
            } else {
                if(jiggleTimer.milliseconds() > 50) {
                    jiggleState = !jiggleState;
                }
            }
            if(jiggleState) {
                y = 1;
            } else {
                y = -1;
            }
            x = 0;
            turn = 0;
        } else if(gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) { // Nudge
            if(gamepad2.dpad_up) {
                y = 0.1;
                x = 0;
                turn = 0;
            } else if (gamepad2.dpad_left) {
                y = 0;
                x = -0.1;
                turn = 0;
            } else if(gamepad2.dpad_right) {
                y = 0;
                x = 0.1;
                turn = 0;
            } else { // Down
                y = -0.1;
                x = 0;
                turn = 0;
            }
            if(dm == DRIVE_MODES.NUDGING) {
                if(nudgeTimer.milliseconds() > 500) {
                    y = 0;
                    x = 0;
                    turn = 0;
                }
            } else {
                dm = DRIVE_MODES.NUDGING;
                nudgeTimer.reset();
            }
        }
         else {
             dm = pdm;
             y = gamepad2.left_stick_y;
             x = -1 * gamepad2.left_stick_x;
             turn = -1 * gamepad2.right_stick_x;
        }
        // Actual drive
        y = (Math.abs(y)>0.05 ? y : 0);
        x = (Math.abs(x)>0.05 ? x : 0);
        turn = (Math.abs(turn)>0.05 ? turn : 0);

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

        double powerMultiplier = modePowers.get(dm);
        r.fl.setPower(MathUtils.clamp(y+x+turn ,-1,1)*powerMultiplier);
        r.fr.setPower(MathUtils.clamp(-(y-x-turn),-1,1)*powerMultiplier);
        r.bl.setPower(MathUtils.clamp(y-x+turn,-1,1)*powerMultiplier);
        r.br.setPower(MathUtils.clamp(-(y+x-turn),-1,1)*powerMultiplier);

        // IMU Reset
        if(gamepad2.back) {
            r.imu.reset();
        }
    }

    public void attachments(){
        // Intake, spin
        r.intake.setPower((gamepad1.a?1:0) - (gamepad1.b?1:0));
        double spinPower = MathUtils.clamp(1.3*(gamepad1.right_trigger-gamepad1.left_trigger),-1,1);
        r.spin0.setPower(spinPower);
        r.spin1.setPower(spinPower);

        //  Lift
        if(gamepad1.dpad_up || (gamepad1.y && r.getLiftPos() < 5000)) {
            r.setLiftPower(1);
        } else if (gamepad1.dpad_down && r.getLiftPos() > 0) {
            r.setLiftPower(-1);
        } else {
            r.setLiftPower(0);
        }

        // Lift Reset
        if(gamepad1.back) {
            r.liftZero = r.lift.getCurrentPosition();
        }

        // Bucket
        if(gamepad1.left_bumper) {
            r.topDumpBucket();
        } else if(gamepad1.right_bumper) {
            r.bottomDumpBucket();
        } else {
            r.restBucket();
        }
    }

}
