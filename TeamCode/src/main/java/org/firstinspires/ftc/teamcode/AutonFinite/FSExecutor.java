package org.firstinspires.ftc.teamcode.AutonFinite;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

import java.util.ArrayList;
import java.util.Scanner;

// Need dictionary for value lookup OR a separate thread for instructions

public class FSExecutor {
    enum DriveStates {
        WAIT, DRIVE_LINEAR, DRIVE_TURN
    }
    enum LiftStates {
        WAIT, MOVE
    }
    enum BucketStates {
        WAIT, MOVE
    }
    enum SpinState {
        FORWARD, REVERSE, OFF
    }

    public ArrayList<String> broadcasts = new ArrayList<>();
    public ArrayList<String> driveInstructions = new ArrayList<>();
    public ArrayList<String> liftInstructions = new ArrayList<>();
    public ArrayList<String> bucketInstructions = new ArrayList<>();
    public ArrayList<String> spinInstructions = new ArrayList<>();

    public volatile boolean outOfInstructions = false;

    protected DriveStates driveS = DriveStates.WAIT;
    protected LiftStates liftS = LiftStates.WAIT;
    protected BucketStates bucketS = BucketStates.WAIT;
    protected SpinState spinS = SpinState.OFF;

    private LinearDriveObject d;
    private TurnObject t;
    private ElapsedTime bucketTimer, spinTimer;
    private int bucketTime, spinTime;

    private Telemetry telemetry;

    private String cDriveInstruction, cLiftInstruction, cBucketInstruction, cSpinInstruction;

    CompBotW2Attachments r;

    public FSExecutor(CompBotW2Attachments r, Telemetry t) {
        this.r = r;
        telemetry = t;
    }

    public void loop() {
        // Drive instructions
        // Format "driveLinear <dForward> <dStrafe> <sForward> <sStrafe> <time>" or "driveTurn <absolute/relative> <angle> <speed> <time>"
        switch(driveS) {
            case WAIT:
                // Check for next instruction
                if(cDriveInstruction.equals("") && !driveInstructions.isEmpty()) {
                    driveInstructions.remove(0);
                    cDriveInstruction = driveInstructions.get(0);
                } else {
                    Scanner ins = new Scanner(cDriveInstruction);
                    String inst = ins.next();
                    if(inst.equals("driveLinear")) {
                        d = new LinearDriveObject(r,ins.nextDouble(),ins.nextDouble(),ins.nextDouble(),ins.nextDouble(),ins.nextInt());
                        driveS = DriveStates.DRIVE_LINEAR;
                    } else if (inst.equals("driveTurn")) {
                        t = new TurnObject(r,ins.next(),ins.nextDouble(),ins.nextDouble(),ins.nextInt());
                        driveS = DriveStates.DRIVE_TURN;
                    } else {
                        if(broadcastSystem(cBucketInstruction)) {
                            cBucketInstruction = "";
                        }
                    }
                }
                break;
            case DRIVE_LINEAR:
                d.DriveLinearCycle();
                if(d.finished) {
                    d.driveFinish();
                    driveS = DriveStates.WAIT;
                }
                break;
            case DRIVE_TURN:
                t.turnCycle();
                if(t.finished) {
                    t.driveFinish();
                    driveS = DriveStates.WAIT;
                }
                break;
        }

        // Lift instructions
        // Format "lift <absolute/relative> <position> <speed>"
        switch(liftS) {
            case WAIT:
                if(cLiftInstruction.equals("") && !liftInstructions.isEmpty()) {
                    liftInstructions.remove(0);
                    cLiftInstruction = liftInstructions.get(0);
                } else {
                    Scanner ins = new Scanner(cLiftInstruction);
                    if(ins.next().equals("lift")) {
                        if(ins.next().equals("absolute")) {
                            r.lift.setTargetPosition(ins.nextInt());
                        } else {
                            r.lift.setTargetPosition(r.lift.getCurrentPosition() + ins.nextInt());
                        }
                        r.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        r.lift.setPower(ins.nextDouble());
                    } else {
                        if(broadcastSystem(cBucketInstruction)) {
                            cBucketInstruction = "";
                        }
                    }
                }
                break;
            case MOVE:
                if(!r.lift.isBusy()) {
                    r.lift.setPower(0);
                }
                liftS = LiftStates.WAIT;
                break;
        }

        // Bucket instructions
        // Format "bucket <position> <timeToWait>"
        switch(bucketS) {
            case WAIT:
                if(cBucketInstruction.equals("") && !bucketInstructions.isEmpty()) {
                    bucketInstructions.remove(0);
                    cBucketInstruction = bucketInstructions.get(0);
                } else {
                    Scanner ins = new Scanner(cBucketInstruction);
                    String inst = ins.next();
                    // Set up spin
                    if(inst.equals("bucket")) {
                        r.setBucket(ins.nextDouble());
                        bucketTime = ins.nextInt();
                        bucketTimer.reset();
                    } else {
                        if(broadcastSystem(cBucketInstruction)) {
                            cBucketInstruction = "";
                        }
                    }
                }
                break;
            case MOVE:
                if(bucketTimer.milliseconds() > bucketTime) {
                    bucketS = BucketStates.WAIT;
                }
                break;
        }

        // Spin instructions
        // Format "spin <direction> <time>"
        if (spinS == SpinState.OFF) {
            if(cSpinInstruction.equals("") && !spinInstructions.isEmpty()) {
                spinInstructions.remove(0);
                cSpinInstruction = spinInstructions.get(0);
            } else {
                Scanner ins = new Scanner(cSpinInstruction);
                String inst = ins.next();
                // Set up spin
                if(inst.equals("spin")) {
                    String direction = ins.next();
                    if(direction.equals("reverse")) {
                        spinS = SpinState.REVERSE;
                    } else {
                        spinS = SpinState.FORWARD;
                    }
                    spinTime = ins.nextInt();
                    spinTimer.reset();
                } else {
                    if(broadcastSystem(cBucketInstruction)) {
                        cBucketInstruction = "";
                    }
                }
            }
        }
        else {
            spinSet();
            if (spinTimer.milliseconds() > spinTime) {
                cSpinInstruction = "";
                spinS = SpinState.OFF;
            }
        }

        if(driveInstructions.isEmpty() && liftInstructions.isEmpty() && bucketInstructions.isEmpty() && spinInstructions.isEmpty()) {
            outOfInstructions = true;
        } else {
            outOfInstructions = false;
        }

        // Telemetry
        telemetry.addLine(String.valueOf(driveS));
        telemetry.addLine(String.valueOf(liftS));
        telemetry.addLine(String.valueOf(bucketS));
        telemetry.addLine(String.valueOf(spinS));
        StringBuilder s = new StringBuilder("broadcasts = {");
        for(String x : broadcasts) {
            s.append(x);
            s.append(", ");
        }
        telemetry.addLine(s + "}");
        telemetry.update();
    }

    private void spinSet() {
        switch(spinS) {
            case FORWARD:
                r.spin0.setPower(CompBotW2Attachments.spinPower);
                r.spin1.setPower(CompBotW2Attachments.spinPower);
                break;
            case REVERSE:
                r.spin0.setPower(-CompBotW2Attachments.spinPower);
                r.spin1.setPower(-CompBotW2Attachments.spinPower);
                break;
            case OFF:
                r.spin0.setPower(0);
                r.spin1.setPower(0);
                break;
        }
    }

    private boolean broadcastSystem(String in) {
        Scanner s = new Scanner(in);
        switch(s.next()) {
            case "send":
                broadcasts.add(s.next());
                return true;
            case "delete":
                broadcasts.remove(s.next());
                return true;
            case "check":
                return broadcasts.contains(s.next());
            default:
                try {
                    throw new Exception("Invalid spin instruction");
                } catch (Exception e) {
                    e.printStackTrace();
                }
                return true;
        }
    }
}
