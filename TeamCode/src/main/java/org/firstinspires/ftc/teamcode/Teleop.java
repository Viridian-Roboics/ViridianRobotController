package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends OpMode {
    enum Drive_Modes {
        ECO_MODE,
        SPORT_MODE,
        N_WORD
    }

    enum Gyro_Modes {
        DRIVE_STRAIGHT,
        TURN
    }

    F0Hardware r = new F0Hardware();
    PID angleCorrector;

    Drive_Modes dm = Drive_Modes.ECO_MODE;
    Gyro_Modes gm = Gyro_Modes.TURN;

    @Override
    public void init() {
        r.init(hardwareMap);
        telemetry.addLine("imu reset in progress...");
        telemetry.update();
        r.imu.reset();
        telemetry.addLine("imu reset finished ^_^");
        telemetry.update();
        angleCorrector = new PID(new double[]{0.05,0,0}, r.imu.getHeading());
        telemetry.addLine("completely finished with init ^o^");
        telemetry.update();
    }
    @Override
    public void loop() {
        double accelInput = gamepad1.right_trigger - gamepad1.left_trigger;
        double steerInput = gamepad1.left_stick_x;

        // Drive mode changing
        if(gamepad1.b) {
            dm = Drive_Modes.ECO_MODE;
        } else if(gamepad1.x) {
            dm = Drive_Modes.SPORT_MODE;
        } else if(gamepad1.y) {
            dm = Drive_Modes.N_WORD;
        }

        // Handbrake
        if(gamepad1.a) {
            accelInput = -1*Math.signum(accelInput);
        }

        double angPowerAdder = 0, lp = 0, rp = 0;

        // Gyro angle correction
        if (gm == Gyro_Modes.DRIVE_STRAIGHT) {
            angPowerAdder = angleCorrector.calculate(r.imu.getHeading());
            lp = accelInput + angPowerAdder;
            rp = accelInput - angPowerAdder;
        } else {
            angleCorrector.setRefVal(r.imu.getHeading());
            lp = accelInput*(1-0.7*steerInput);
            rp = accelInput*(1-0.7*steerInput);
            if(Math.abs(steerInput) < 0.03) {
                gm = Gyro_Modes.DRIVE_STRAIGHT;
            }
        }

        switch(dm) {
            case ECO_MODE:
                lp *= 0.25;
                rp *= 0.25;
                telemetry.addLine("Eco Mode");
                break;
            case SPORT_MODE:
                lp *= 0.6;
                rp *= 0.6;
                telemetry.addLine("Sport Mode");
                break;
            case N_WORD:
                telemetry.addLine("N Word");
                break;
        }

        r.left.setPower(lp);
        r.right.setPower(rp);

        // Steering
        InterpLUT servoPosition = new InterpLUT();
        servoPosition.add(-1.01,-1);
        servoPosition.add(0,0);
        servoPosition.add(1.01, 1);
        servoPosition.createLUT();
        InterpLUT speedCorrection = new InterpLUT();
        speedCorrection.add(-0.01,1);
        speedCorrection.add(1.01, 0.2);
        speedCorrection.createLUT();
        double tAng = servoPosition.get(steerInput)*speedCorrection.get(Math.abs(steerInput));
        r.steer.turnToAngle((tAng));

        telemetry.update();
    }
    @Override
    public void stop() {
        r.stop();
    }
}
