package org.firstinspires.ftc.teamcode.CompBotW2;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CompBotW3.CompBotW3Attachments;

import java.util.Arrays;

public class CompBotW2Attachments extends CompBotW2 {
    enum Bucket_States {
        REST, TEMP, BOTTOM_DUMP, TOP_DUMP;
    }

    public Bucket_States bs = Bucket_States.REST;

    public DcMotor intake = null, lift = null;

    public Servo bucket0, bucket1, azalServo;
    public CRServo spin0, spin1;

    public final static int liftMaxAdder = 10000, lowLift = 1400, medLift = 3600, highLift = 6300;
    public int liftZero, liftSafe, liftMax;
    public final static double spinPower = 0.3;

    private int liftHold;
    private boolean holding = false;

    public CompBotW2Attachments() {
        super();
    }

    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        thisInit(hardwareMap);
    }
    public void init(HardwareMap hardwareMap, boolean cameraInit, Telemetry telemetry) {
        if(cameraInit) {
            super.init(hardwareMap, cameraInit, telemetry);
        }
        thisInit(hardwareMap);
    }
    public void init(HardwareMap hardwareMap, boolean cameraInit, Telemetry telemetry, String color) {
        if(cameraInit) {
            super.init(hardwareMap, cameraInit, telemetry, color);
        }
        thisInit(hardwareMap);
    }
    public void thisInit(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        spin0 = hardwareMap.get(CRServo.class, "spin0");
        spin1 = hardwareMap.get(CRServo.class, "spin1");
        lift = hardwareMap.get(DcMotor.class, "lift");
        bucket0 = hardwareMap.get(Servo.class, "bucket0");
        bucket1 = hardwareMap.get(Servo.class, "bucket1");
        azalServo = hardwareMap.get(Servo.class, "azalServo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.REVERSE);

        bucket0.setDirection(Servo.Direction.FORWARD);
        bucket1.setDirection(Servo.Direction.REVERSE);
        azalServo.setDirection(Servo.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setPower(0);
        spin0.setPower(0);
        spin1.setPower(0);
        lift.setPower(0);
        resetLift();
    }

    public void stop() {
        super.stop();
        intake.setPower(0);
        lift.setPower(0);
    }
    public void resetLift() {
        lift.setPower(0);
        liftZero = lift.getCurrentPosition();
        liftMax = liftSafe+liftMaxAdder;
    }
    public int getLiftPos() {
        return lift.getCurrentPosition()-liftZero;
    }
    public void setLiftPower(double p) {
        lift.setPower(p);
    }
    public void safeLiftDrive(double p) {
        if((lift.getCurrentPosition() > liftMax && Math.signum(p) == 1) || (lift.getCurrentPosition() < liftZero && Math.signum(p) == -1)) {
            lift.setPower(0);
        } else {
            lift.setPower(p);
        }
    }
    public void moveLiftPosition(int ticksToDrive, double speed) {
        lift.setTargetPosition(lift.getCurrentPosition() + ticksToDrive);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(lift.isBusy()) {
            lift.setPower(speed);
        }
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setLiftPosition(int pos, double speed) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(lift.isBusy()) {
            lift.setPower(speed);
        }
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean isLiftSafe() {
        return lift.getCurrentPosition() >= liftSafe && lift.getCurrentPosition() <= liftMax;
    }

    public void setBucket(double position) {
        bucket0.setPosition(position);
        bucket1.setPosition(position);
    }
    public void restBucket() {
        setBucket(0.67);
        bs = Bucket_States.REST;
    }
    public void tempBucket() {
        setBucket(0.1);
        bs = Bucket_States.TEMP;
    }
    public void topDumpBucket() {
        setBucket(0);
        bs = Bucket_States.BOTTOM_DUMP;
    }
    public void bottomDumpBucket() {
        setBucket(1);
        bs = Bucket_States.TOP_DUMP;
    }

    public void spin(long time) {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < time) {
            spin0.setPower(spinPower);
            spin1.setPower(spinPower);
        }
        spin0.setPower(0);
        spin1.setPower(0);
    }
    public void spinReverse(long time) {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < time) {
            spin0.setPower(-spinPower);
            spin1.setPower(-spinPower);
        }
        spin0.setPower(0);
        spin1.setPower(0);
    }
    public void setSpin(double power) {
        spin0.setPower(power);
        spin1.setPower(power);
    }

    public void lowLift() {
        setLiftPosition(liftZero+lowLift,1);
    }
    public void medLift() {
        setLiftPosition(liftZero+medLift, 1);
    }
    public void highLift() {
        setLiftPosition(liftZero+highLift, 1);
    }
    public void zeroLift() {
        setLiftPosition(liftZero, 1);
    }

    public void autonLift(boolean[] p) {
        highLift();
        tempBucket();
        sleep(250);
        if (Arrays.equals(p, new boolean[]{true, false, false})) {// left
            lowLift();
        } else if (Arrays.equals(p, new boolean[]{false, true, false})) {// middle
            medLift();
        } else {// right
            highLift();
        }
        topDumpBucket();
        sleep(500);
        highLift();
        restBucket();
        sleep(250);
        zeroLift();

    }

    private void sleep(int i) {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < i) {

        }
    }
}