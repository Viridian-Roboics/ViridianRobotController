package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

@TeleOp(name="Wheel testing program")
public class wheeltest extends LinearOpMode {
    CompBotW2Attachments r= new CompBotW2Attachments();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);

        waitForStart();

        ElapsedTime x = new ElapsedTime();

        while(x.milliseconds() < 1000) {
            r.fl.setPower(0.3);
        }
        r.fl.setPower(0);
        x.reset();
        while(x.milliseconds() < 1000) {
            r.bl.setPower(0.3);
        }
        r.bl.setPower(0);
        x.reset();
        while(x.milliseconds() < 1000) {
            r.fr.setPower(0.3);
        }
        r.fr.setPower(0);
        x.reset();
        while(x.milliseconds() < 1000) {
            r.br.setPower(0.3);
        }
        r.br.setPower(0);
    }
}
