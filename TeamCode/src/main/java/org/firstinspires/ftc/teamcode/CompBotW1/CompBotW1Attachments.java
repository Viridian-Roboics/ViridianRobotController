package org.firstinspires.ftc.teamcode.CompBotW1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class CompBotW1Attachments extends CompBotW1 {
    public DcMotor intake = null, lift = null;

    public Servo bucket0, bucket1;
    public CRServo spin0, spin1;

    public final static int liftSafeAdder = 6300, liftMaxAdder = 10000;
    public int liftZero, liftSafe, liftMax;
    public final static double spinPower = -1;
    public double bucketOpen = 0.25, bucketClosed = 1;

    public final static int lowLift = 1400, medLift = 3600, highLift = 6300;

    private int liftHold;
    private boolean holding = false;

    public CompBotW1Attachments() {
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

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.REVERSE);

        bucket0.setDirection(Servo.Direction.FORWARD);
        bucket1.setDirection(Servo.Direction.FORWARD);
        setBucket(1);

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
        liftSafe = liftZero+liftSafeAdder;
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
        bucket1.setPosition(1-position);
    }
    public double BucketPosition(){
        return bucket0.getPosition();
    }
    public void ShareGoal(){
        if (getLiftPos() > 5000) { //overflow
            int dif = (getLiftPos() - 5000);
            moveLiftPosition(dif, 1);
        }
        else if (getLiftPos() < 5000) {
            int dif = -1 * (getLiftPos() - 5000);
            moveLiftPosition(dif, -1);
        }
        setBucket(.5);
        moveLiftPosition(-1700,-1);
        setBucket(.3);
        moveLiftPosition(1700, 1);
        setBucket(1);
        moveLiftPosition(-5000, -1);
    }
    public void fixBucket() {
        setLiftPosition(liftSafe, 1);
        setBucket(0.45);
    }

    public void spin(long time) {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < time) {
            spin0.setPower(spinPower * .7);
            spin1.setPower(spinPower * .7);
        }
        spin0.setPower(0);
        spin1.setPower(0);
    }

    public void spin2(long time) {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < time) {
            spin0.setPower(-spinPower * .7);
            spin1.setPower(-spinPower * .7);
        }
        spin0.setPower(0);
        spin1.setPower(0);
    }

    public void spinReverse(long time) {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < time) {
            spin0.setPower(-1*spinPower);
            spin1.setPower(-1*spinPower);
        }
        spin0.setPower(0);
        spin1.setPower(0);
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

    public void autonLift(boolean[] p, double dPower) {

        AEncDrive(-3,0,-0.15,0,2000);
        AEncDrive(20,0,dPower,0);
        fixBucket();
        sleep(250);
        if (Arrays.equals(p, new boolean[]{true, false, false})) {// left
            lowLift();
        } else if (Arrays.equals(p, new boolean[]{false, true, false})) {// middle
            medLift();
        } else {// right
            highLift();
        }
        setBucket(0.25);
        sleep(500);
        fixBucket();
        setBucket(1);
        sleep(250);
        zeroLift();
        AEncDrive(-22,0,dPower,0,2000);
    }

    private void sleep(int i) {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < i) {

        }
    }
}