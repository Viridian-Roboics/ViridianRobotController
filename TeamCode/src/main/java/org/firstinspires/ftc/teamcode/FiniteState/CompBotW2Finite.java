package org.firstinspires.ftc.teamcode.FiniteState;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2;
import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

public class CompBotW2Finite extends CompBotW2Attachments {
    public final double liftSpeed = 1;

    private ElapsedTime liftTimer = new ElapsedTime();
    private boolean resetLift = false;

    public void init(HardwareMap h) {
        super.init(h);
    }
    public void init(HardwareMap h, boolean cameraInit, Telemetry telemetry, String color) {
        super.init(h,cameraInit,telemetry,color);
    }

    // Cycle-based PID drive
    public void setDriveOffset(int a, int b, int c, int d) {
        fl.setTargetPosition(fl.getCurrentPosition() + a);
        fr.setTargetPosition(fr.getCurrentPosition() + b);
        bl.setTargetPosition(bl.getCurrentPosition() + c);
        br.setTargetPosition(br.getCurrentPosition() + d);
        runToPositionMode(); // Set motors to RUN_TO_POSITION mode - they will automatically spin in the direction of the set position
    }
    public void setDriveInches(double dForward, double dStrafe) {
        setDriveOffset((int) -(distanceK*(dForward+dStrafe)), (int) (distanceK*(dForward-dStrafe)), (int) -(distanceK*(dForward-dStrafe)), (int) (distanceK*(dForward+dStrafe)));
    }
    public void DriveCycle(double sForward, double sStrafe, double totalError) {
        if(fl.isBusy()) { fl.setPower(-(sForward + sStrafe + totalError)); // DCMotor.isBusy is a boolean variable signifying whether the motor has finished moving to the position
        } else { fl.setPower(MathUtils.clamp((fl.getCurrentPosition()-fl.getTargetPosition() < 0 ? -1 : 1)*totalError, -1, 1));
        }if(fr.isBusy()) { fr.setPower(sForward - sStrafe - totalError); // This code looks complicated but it's simple
        } else { fr.setPower(MathUtils.clamp((fr.getCurrentPosition()-fr.getTargetPosition() < 0 ? -1 : 1)*-1*totalError,-1,1));
        }if(bl.isBusy()) { bl.setPower(-(sForward - sStrafe + totalError)); // If the motor is not finished, apply the given speed + a correction based on the angle error
        } else { bl.setPower(MathUtils.clamp((bl.getCurrentPosition()-bl.getTargetPosition() < 0 ? -1 : 1)*totalError,-1,1));
        } if(br.isBusy()) { br.setPower(sForward + sStrafe - totalError); // If the motor is finished, apply only the correction (flip flops signs because we're in RUN_TO_POSITION mode)
        } else { br.setPower(MathUtils.clamp((br.getCurrentPosition()-br.getTargetPosition() < 0 ? -1 : 1)*-1*totalError,-1,1));
        }
    }
    public void driveFinish() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        useEncoders(); // Switch back to normal RUN_USING_ENCODERS velocity control mode
    }
    /*
    public void AEncSimplified(double dForward, double dStrafe, double sForward, double sStrafe, long time) {
        // Set the target positions of each motor
        setDriveInches(dForward, dStrafe);
        PID corrector = new PID(new double[]{CompBotW2.corrCoeff,0,0},imu.getHeading());
        ElapsedTime total = new ElapsedTime();
        while((fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) && total.milliseconds() < time) {
            double totalError = corrector.calculate(imu.getHeading());
            DriveCycle(sForward, sStrafe, totalError);
        }

    }
     */

    // Cycle-based lift movement
    public void setLiftEncoderRelative(int position) {
        lift.setTargetPosition(lift.getCurrentPosition() + position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setLiftEncoderAbsolute(int position) {
        lift.setTargetPosition(position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void liftCycle() {
        if(lift.isBusy()) {
            lift.setPower(liftSpeed);
        }
    }
    public void liftFinish() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void liftMoveRelative(int position) {
        setLiftEncoderRelative(position);
        while (lift.isBusy()) {
            liftCycle();
        }
        liftFinish();
    }
    public void liftMoveAbsolute(int position) {
        setLiftEncoderAbsolute(position);
        while (lift.isBusy()) {
            liftCycle();
        }
        liftFinish();
    }

    // Bucket cooldown
    public void setBucketCooldown(double position) {
        if(!resetLift || liftTimer.milliseconds() > 500) {
            setBucket(position);
        }
        resetLift = true;
    }
}
