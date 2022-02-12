package org.firstinspires.ftc.teamcode.Teleops;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;
import org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3;

@TeleOp
public class DemoOp extends OpMode {
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
        r.intake.setPower((gamepad1.a?1:0) - (gamepad1.b?1:0));
        r.spin0.setPower(gamepad1.left_stick_x);
        r.spin1.setPower(gamepad1.left_stick_x);

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
