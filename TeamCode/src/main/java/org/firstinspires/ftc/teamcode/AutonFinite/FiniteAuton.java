package org.firstinspires.ftc.teamcode.AutonFinite;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

import java.util.ArrayList;
import java.util.Scanner;

// Need dictionary for value lookup OR a separate thread for instructions

public class FiniteAuton extends LinearOpMode {
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

    private ArrayList<String> broadcasts = new ArrayList<>();

    private DriveStates driveS = DriveStates.WAIT;
    private LiftStates liftS = LiftStates.WAIT;
    private BucketStates bucketS = BucketStates.WAIT;
    private SpinState spinS = SpinState.OFF;

    private LinearDriveObject d;
    private TurnObject t;
    private ElapsedTime bucketTimer, spinTimer;
    private int bucketTime, spinTime;

    private Scanner driveInstructions = new Scanner();
    private Scanner liftInstructions = new Scanner();
    private Scanner bucketInstructions = new Scanner();
    private Scanner spinInstructions = new Scanner();
    private String cDriveInstruction, cLiftInstruction, cBucketInstruction, cSpinInstruction;

    CompBotW2Attachments r = new CompBotW2Attachments();

    ElapsedTime runtime = new ElapsedTime();
    boolean breakLoop = false;

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true, telemetry,"blue");
        telemetry.addLine("init finished");
        telemetry.update();
        boolean[] pos = {false,false,false};
        ElapsedTime e = new ElapsedTime();
        while(!isStarted()) {
            pos = r.p.getPositions(); // Detection
        }
        r.phoneCam.stopStreaming();
        r.setBucket(1);
        double heading = r.imu.getHeading();

        runtime.reset();

        while(opModeIsActive() && !breakLoop) {
            // Drive instructions
            // Format "driveLinear <dForward> <dStrafe> <sForward> <sStrafe> <time>" or "driveTurn <absolute/relative> <angle> <speed> <time>"
            switch(driveS) {
                case WAIT:
                    // Check for next instruction
                    if(cDriveInstruction.equals("")) {
                        cDriveInstruction = driveInstructions.nextLine();
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
                    if(cLiftInstruction.equals("")) {
                        cLiftInstruction = liftInstructions.nextLine();
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
                    if(cBucketInstruction.equals("")) {
                        cBucketInstruction = bucketInstructions.nextLine();
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
                if (cSpinInstruction.equals("")) {
                    cSpinInstruction = spinInstructions.nextLine();
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

            // Check if auton is finished
            if(!(driveInstructions.hasNext() && liftInstructions.hasNext() && bucketInstructions.hasNext() && spinInstructions.hasNext())) {
                breakLoop = true;
            }
            else {
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
        }

        r.stop();

        telemetry.addData("Runtime",runtime.milliseconds());
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
