package org.firstinspires.ftc.teamcode.Disabled;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;
import org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3;

@TeleOp(name="Viridian Competition Teleop old")
@Disabled
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
        r.spin0.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
        r.spin1.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

        if((gamepad1.right_trigger > 0.2 && r.getLiftPos() < 6430) || gamepad1.dpad_up) {
            r.setLiftPower(liftPower);
        } else if ((gamepad1.left_trigger > 0.2 && r.getLiftPos() > 0) || gamepad1.dpad_down) {
            r.setLiftPower(-liftPower);
        } else {
            r.setLiftPower(0);
        }

        if(gamepad1.right_stick_button) {
            r.imu.reset();
        }

        r.setBucket(gamepad1.left_bumper?.25:1);

        telemetry.addLine(String.valueOf(gamepad1.a));
        telemetry.addData("BucketPos: ", r.BucketPosition());
        telemetry.addData("liftPos: ", r.getLiftPos());
        telemetry.update();

        if(gamepad1.x) {
            r.liftZero = r.lift.getCurrentPosition();
        }


    }

    @Override
    public void stop() {
        r.stop();
    }
}
