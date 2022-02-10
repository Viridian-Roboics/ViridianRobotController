package org.firstinspires.ftc.teamcode.AutonFinite;

import androidx.core.math.MathUtils;


import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

public class LinearDriveObject extends DriveObject {
    private final double dForward, dStrafe, sForward, sStrafe;
    private final int time;
    private final double[] coeff = {CompBotW2Attachments.corrCoeff, 0,0};

    private double initial, pastError = 0, intError = 0;

    public boolean finished = false;

    ElapsedTime e = new ElapsedTime();
    ElapsedTime total = new ElapsedTime();

    public LinearDriveObject(CompBotW2Attachments r, double dForward, double dStrafe, double sForward, double sStrafe, int time) {
        super(r);
        this.dForward = dForward;
        this.dStrafe = dStrafe;
        this.sForward = sForward;
        this.sStrafe = sStrafe;
        this.time = time;
        r.fl.setTargetPosition(r.fl.getCurrentPosition() + (int) -(CompBotW2Attachments.distanceK*(dForward+dStrafe)));
        r.fr.setTargetPosition(r.fr.getCurrentPosition() + (int) (CompBotW2Attachments.distanceK*(dForward-dStrafe)));
        r.bl.setTargetPosition(r.bl.getCurrentPosition() + (int) -(CompBotW2Attachments.distanceK*(dForward-dStrafe)));
        r.br.setTargetPosition(r.br.getCurrentPosition() + (int) (CompBotW2Attachments.distanceK*(dForward+dStrafe)));
        r.runToPositionMode(); // Set motors to RUN_TO_POSITION mode - they will automatically spin in the direction of the set position
        initial = r.imu.getHeading();
        e.reset();
        total.reset();
    }
    public void DriveLinearCycle() {
        double eTime = e.milliseconds();
        double error = r.imu.getHeading() - initial;
        double dervError = (error - pastError)/eTime;
        intError += error*eTime;
        e.reset();
        double totalError = coeff[0]*error + coeff[1]*intError + coeff[2]*dervError;
        pastError = error;
        if(r.fl.isBusy()) { r.fl.setPower(-(sForward + sStrafe + totalError)); // DCMotor.isBusy is a boolean variable signifying whether the motor has finished moving to the position
        } else { r.fl.setPower(MathUtils.clamp((r.fl.getCurrentPosition()-r.fl.getTargetPosition() < 0 ? -1 : 1)*totalError, -1, 1));
        }if(r.fr.isBusy()) { r.fr.setPower(sForward - sStrafe - totalError); // This code looks complicated but it's simple
        } else { r.fr.setPower(MathUtils.clamp((r.fr.getCurrentPosition()-r.fr.getTargetPosition() < 0 ? -1 : 1)*-1*totalError,-1,1));
        }if(r.bl.isBusy()) { r.bl.setPower(-(sForward - sStrafe + totalError)); // If the motor is not finished, apply the given speed + a correction based on the angle error
        } else { r.bl.setPower(MathUtils.clamp((r.bl.getCurrentPosition()-r.bl.getTargetPosition() < 0 ? -1 : 1)*totalError,-1,1));
        } if(r.br.isBusy()) { r.br.setPower(sForward + sStrafe - totalError); // If the motor is finished, apply only the correction (flip flops signs because we're in RUN_TO_POSITION mode)
        } else { r.br.setPower(MathUtils.clamp((r.br.getCurrentPosition()-r.br.getTargetPosition() < 0 ? -1 : 1)*-1*totalError,-1,1));
        }
        if(!((r.fl.isBusy() || r.fr.isBusy() || r.bl.isBusy() || r.br.isBusy()) && total.milliseconds() < time)) {
            finished = true;
        }
    }
}
