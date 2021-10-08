package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="F0 Traction Control Drive")
@Disabled

public class F0tractionDrive extends OpMode {
    private double slipK = 0.2;
    private final double turnK = 0.2;
    private boolean tcON = true;

    F0Advanced f = new F0Advanced();

    @Override
    public void init() {
        f.init(hardwareMap);
        f.imu.getRevIMU().startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop() {
        // Gather controls
        double accel = gamepad1.right_trigger - gamepad1.left_trigger;
        double turn = gamepad1.left_stick_x;

        // Configure traction control
        if(gamepad1.dpad_right) {
            if(slipK < 0.4) {
                slipK += 0.01;
            } else {
                tcON = false;
            }
        } else if(gamepad1.dpad_left) {
            if(!tcON) {
                tcON = true;
            } else if(slipK > 0) {
                slipK -= 0.01;
            }
        }
        // Push to telemetry
        if(tcON) {
            telemetry.addData("Slip coefficient", slipK);
        } else {
            telemetry.addLine("Traction control off");
        }

        // Steering
        InterpLUT servoPosition = new InterpLUT();
        servoPosition.add(-1,-10);
        servoPosition.add(0,0);
        servoPosition.add(1, 10);
        servoPosition.createLUT();
        double tAng = servoPosition.get(turn);
        f.steer.turnToAngle(tAng);
        telemetry.addData("Servo angle", tAng);

        // Driving

        // Find magnitude of linear velocity and angular velocity
        double magVel = f.getMagVxy(); //* NEEDS TO BE CONVERTED TO M/S!!!!! *//
        double angVel = f.getAngVel();
        telemetry.addData("Magnitude of linear velocity", magVel);
        telemetry.addData("Angular velocity in radians/sec", angVel);

        // Find the commanded powers by the gamepad
        double driveX = 0.25*accel;
        double driveRot  = turnK*magVel*turn;
        double[] commandedPower = {driveX - driveRot, driveX + driveRot};
        telemetry.addData("Commanded power", commandedPower);

        if(tcON) {
            // Find the wheel speeds of each wheel in meters/second
            double[] wSpeed = {F0Advanced.mps(F0Advanced.getWheelSpeed(f.lEnc)), F0Advanced.mps(F0Advanced.getWheelSpeed(f.rEnc))};

            // Finds the component of the wheel speed that causes the robot to turn
            double leftRotWSComp = angVel*F0Advanced.robotWidthM;
            double rightRotWSComp = -1*angVel*F0Advanced.robotWidthM;

            // Finds the overall expected wheel speed
            double[] expSpeed = {magVel + leftRotWSComp, magVel - rightRotWSComp};
            double[] slip = new double[commandedPower.length];

            for(int i = 0; i < commandedPower.length; i++) {
                // Find the slip % = error of actual speed from expected speed
                slip[i] = Math.abs((expSpeed[i] - wSpeed[i])/expSpeed[i]);
                // If slipping between min and max slip, apply a proportional correction, otherwise cut power entirely
                if(slip[i] > 2*slipK) {
                    commandedPower[i] = 0;
                } else if (slipK < slip[i]) {
                    // Linear function with points (minSlipK, initial commandedPower[i]) and (maxSlipK, 0)
                    commandedPower[i] -= (slip[i] - 2*slipK)/(2*slipK - slipK);
                }
            }

            telemetry.addData("Slip", slip);
            telemetry.addData("Actual power", commandedPower);
        }

        // Apply powers
        f.left.setPower(commandedPower[0]);
        f.right.setPower(commandedPower[1]);

        telemetry.update();
    }

    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
