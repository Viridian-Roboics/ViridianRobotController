package org.firstinspires.ftc.teamcode.tests;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2;
import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;
import org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3;

@TeleOp
public class DrivingDistanceTest extends OpMode {
    private final double dK = 384.5/(100*Math.PI)*25.4;

    CompBotW2Attachments r = new CompBotW2Attachments();
    double initialHeading, error;
    boolean headingReset = false;

    int flZ = 0, frZ = 0, blZ = 0, brZ = 0;

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        r.setBucket(1);
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

        if(gamepad1.a) {
            flZ = r.fl.getCurrentPosition();
            frZ = r.fr.getCurrentPosition();
            blZ = r.bl.getCurrentPosition();
            brZ = r.br.getCurrentPosition();
        }


        // Telemetry
        telemetry.addData("fl",(r.fl.getCurrentPosition()-flZ)/dK);
        telemetry.addData("fr",(r.fr.getCurrentPosition()-frZ)/dK);
        telemetry.addData("bl",(r.bl.getCurrentPosition()-blZ)/dK);
        telemetry.addData("br",(r.br.getCurrentPosition()-brZ)/dK);
        telemetry.addData("h",r.imu.getHeading());
        telemetry.update();

    }
}
