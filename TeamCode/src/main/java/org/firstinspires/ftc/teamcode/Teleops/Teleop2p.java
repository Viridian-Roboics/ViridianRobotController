package org.firstinspires.ftc.teamcode.Teleops;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;
import org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3;

@TeleOp(name="Viridian Competition Teleop")
public class Teleop2p extends OpMode {
    CompBotW1Attachments r = new CompBotW1Attachments();
    double initialHeading, error;
    boolean headingReset = false;

    double liftPower = 1;

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        double y, x, turn;
        if(Math.abs(gamepad2.left_stick_y) > 0 || Math.abs(gamepad2.left_stick_x) > 0 ||  Math.abs(gamepad2.right_stick_x) > 0) {
            y = gamepad2.left_stick_y;
            x = -1*gamepad2.left_stick_x;
            turn = -1*gamepad2.right_stick_x;
        } else {
            y = gamepad1.left_stick_y;
            x = -1*gamepad1.left_stick_x;
            turn = -1*gamepad1.right_stick_x;
        }

        // Deadzone
        y = (Math.abs(y)>0.05 ? y : 0);
        x = (Math.abs(x)>0.05 ? x : 0);
        turn = (Math.abs(turn)>0.05 ? turn : 0);

        // Power adjust
        y *= (gamepad1.right_stick_button || gamepad2.right_bumper ?0.4:1);
        x *= (gamepad1.right_stick_button || gamepad2.right_bumper ?0.4:1);
        turn *= (gamepad1.right_stick_button || gamepad2.right_bumper ?0.4:1);

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
//            r.ShareGoal();
//            telemetry.addData("done: ", r.getLiftPos());
//            telemetry.update();
            if (r.getLiftPos() > 5000) { //overflow
                int dif = (r.getLiftPos() - 5000);
                r.moveLiftPosition(dif, 1);
            }
            else if (r.getLiftPos() < 5000) {
                int dif = -1 * (r.getLiftPos() - 5000);
                r.moveLiftPosition(dif, -1);
            }
            r.setBucket(.5);
            r.moveLiftPosition(-1700,-1);
            r.setBucket(.3);
            r.moveLiftPosition(1700, 1);
            r.setBucket(1);
            r.moveLiftPosition(-5000, -1);
        }
        telemetry.addData("BucketPos: ", r.BucketPosition());
        telemetry.addData("liftPos: ", r.getLiftPos());
        telemetry.update();
    }

    @Override
    public void stop() {
        r.stop();
    }
}
