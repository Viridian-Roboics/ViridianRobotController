package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ahjdssdjakhsduhgdwquh9nqwubdwqubuwbdqubiwdubwdniqdwbnuji")
public class MotorTest extends OpMode {
    public DcMotor fl = null, fr = null, bl = null, br = null;
    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        fl.setPower(1);
        fr.setPower(1);
        bl.setPower(1);
        br.setPower(1);

        ElapsedTime dt = new ElapsedTime();
        double oldPos, newPos;
        oldPos = fl.getCurrentPosition();
        dt.reset();
        while(dt.milliseconds() < 10) {

        }
        newPos = fl.getCurrentPosition();
        double vel = (newPos - oldPos)/dt.milliseconds();
        telemetry.addLine("Speed of fl motor: "+vel+" clicks/ms");
        oldPos = fr.getCurrentPosition();
        dt.reset();
        while(dt.milliseconds() < 10) {

        }
        newPos = fr.getCurrentPosition();
        vel = (newPos - oldPos)/dt.milliseconds();
        telemetry.addLine("Speed of fr motor: "+vel+" clicks/ms");
        oldPos = bl.getCurrentPosition();
        dt.reset();
        while(dt.milliseconds() < 10) {

        }
        newPos = bl.getCurrentPosition();
        vel = (newPos - oldPos)/dt.milliseconds();
        telemetry.addLine("Speed of bl motor: "+vel+" clicks/ms");
        oldPos = br.getCurrentPosition();
        dt.reset();
        while(dt.milliseconds() < 10) {

        }
        newPos = br.getCurrentPosition();
        vel = (newPos - oldPos)/dt.milliseconds();
        telemetry.addLine("Speed of br motor: "+vel+" clicks/ms");
        telemetry.update();
    }
}
