package org.firstinspires.ftc.teamcode.CompBotW2;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SimpleVisionYCbCr;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CompBotW2 {

    public final static double distanceK = 384.5/(100*Math.PI)*25.4, corrCoeff = 0.05, corrCoeff2 = 1, dervCoeff = 0, intCoeff = 0;

    public final Scalar[] blueColor = {new Scalar(0,101,140,0), new Scalar(255,125,175,255)};
    public final Scalar[] redColor = {new Scalar(0,172,98,0), new Scalar(255,196,116,255)};
    public final Scalar[] capstoneColor = {new Scalar(0,80,75,0), new Scalar(255,95,107,255)};


    public DcMotor fl = null, fr = null, bl = null, br = null;
    public RevIMU imu = null;

    public int cameraMonitorViewId;
    public OpenCvCamera phoneCam = null;
    public SimpleVisionYCbCr p;

    public CompBotW2() {}

    public void init(HardwareMap hardwareMap) {
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = new RevIMU(hardwareMap,"imu");
        imu.init(parameters);
    }
    public void init(HardwareMap h, boolean cameraInit, Telemetry telemetry, String color) {
        if(cameraInit) {
            cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(h.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            if(color.equals("blue")) {
                p = new SimpleVisionYCbCr(telemetry, blueColor, capstoneColor);
            } else if (color.equals("red")) {
                p = new SimpleVisionYCbCr(telemetry, redColor, capstoneColor);
            }
            phoneCam.setPipeline(p);

            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened() { phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); }

                //@Override
                public void onError(int errorCode) {}
            });
        }
        init(h);
    }
    public void init(HardwareMap h, boolean cameraInit, Telemetry telemetry) {
        init(h,cameraInit,telemetry,"blue");
    }
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void AEncDrive(double dForward, double dStrafe, double sForward, double sStrafe, long time) { // d = distance, s = speed
        // Set the target positions of each motor
        fl.setTargetPosition(fl.getCurrentPosition() + (int) -(distanceK*(dForward+dStrafe))); // distanceK is a conversion factor to convert linear distance to motor clicks;
        fr.setTargetPosition(fr.getCurrentPosition() + (int) (distanceK*(dForward-dStrafe))); // The distance each wheel needs to travel is just the sum of the
        bl.setTargetPosition(bl.getCurrentPosition() + (int) -(distanceK*(dForward-dStrafe))); // distances the wheel would need to travel to do the strafing and
        br.setTargetPosition(br.getCurrentPosition() + (int) (distanceK*(dForward+dStrafe))); // forward/back distances separately
        runToPositionMode(); // Set motors to RUN_TO_POSITION mode - they will automatically spin in the direction of the set position
        double initialHeading = imu.getHeading(); // Create variables for the gyro correction, and measure the initial angle the robot is facing
        double pastError = 0, intError = 0;
        ElapsedTime e = new ElapsedTime();
        ElapsedTime total = new ElapsedTime();
        while((fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) && total.milliseconds() < time) {
            double elapsedTime = e.milliseconds();
            e.reset(); // Reset the timer
            double error = imu.getHeading() - initialHeading; // Calculate the deviation from the initial angle of the robot using the gyro
            double dervError = (error-pastError)/elapsedTime; // Calculate the derivative = rate of change of the error
            intError += error*elapsedTime; // Calculate the integral = sum over time of error
            double totalError = (corrCoeff*error + dervCoeff*dervError + intCoeff*intError); // Sum the errors and apply coefficients
            if(fl.isBusy()) { fl.setPower(-(sForward + sStrafe + totalError)); // DCMotor.isBusy is a boolean variable signifying whether the motor has finished moving to the position
            } else { fl.setPower(MathUtils.clamp((fl.getCurrentPosition()-fl.getTargetPosition() < 0 ? -1 : 1)*totalError, -1, 1));
            }if(fr.isBusy()) { fr.setPower(sForward - sStrafe - totalError); // This code looks complicated but it's simple
            } else { fr.setPower(MathUtils.clamp((fr.getCurrentPosition()-fr.getTargetPosition() < 0 ? -1 : 1)*-1*totalError,-1,1));
            }if(bl.isBusy()) { bl.setPower(-(sForward - sStrafe + totalError)); // If the motor is not finished, apply the given speed + a correction based on the angle error
            } else { bl.setPower(MathUtils.clamp((bl.getCurrentPosition()-bl.getTargetPosition() < 0 ? -1 : 1)*totalError,-1,1));
            } if(br.isBusy()) { br.setPower(sForward + sStrafe - totalError); // If the motor is finished, apply only the correction (flip flops signs because we're in RUN_TO_POSITION mode)
            } else { br.setPower(MathUtils.clamp((br.getCurrentPosition()-br.getTargetPosition() < 0 ? -1 : 1)*-1*totalError,-1,1));
            }
            pastError = error; // Move error into pastError for next loop
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        useEncoders(); // Switch back to normal RUN_USING_ENCODERS velocity control mode
    }
    public void AEncDrive(double dForward, double dStrafe, double sForward, double sStrafe, long time, Telemetry telemetry) { // d = distance, s = speed
        // Set the target positions of each motor
        fl.setTargetPosition(fl.getCurrentPosition() + (int) -(distanceK*(dForward+dStrafe))); // distanceK is a conversion factor to convert linear distance to motor clicks;
        fr.setTargetPosition(fr.getCurrentPosition() + (int) (distanceK*(dForward-dStrafe))); // The distance each wheel needs to travel is just the sum of the
        bl.setTargetPosition(bl.getCurrentPosition() + (int) -(distanceK*(dForward-dStrafe))); // distances the wheel would need to travel to do the strafing and
        br.setTargetPosition(br.getCurrentPosition() + (int) (distanceK*(dForward+dStrafe))); // forward/back distances separately
        runToPositionMode(); // Set motors to RUN_TO_POSITION mode - they will automatically spin in the direction of the set position
        double initialHeading = imu.getHeading(); // Create variables for the gyro correction, and measure the initial angle the robot is facing
        double pastError = 0, intError = 0;
        ElapsedTime e = new ElapsedTime();
        ElapsedTime total = new ElapsedTime();
        while((fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) && total.milliseconds() < time) {
            double elapsedTime = e.milliseconds();
            e.reset(); // Reset the timer
            double error = imu.getHeading() - initialHeading; // Calculate the deviation from the initial angle of the robot using the gyro
            double dervError = (error-pastError)/elapsedTime; // Calculate the derivative = rate of change of the error
            intError += error*elapsedTime; // Calculate the integral = sum over time of error
            double totalError = (corrCoeff*error + dervCoeff*dervError + intCoeff*intError); // Sum the errors and apply coefficients
            if(fl.isBusy()) { fl.setPower(-(sForward + sStrafe + totalError)); // DCMotor.isBusy is a boolean variable signifying whether the motor has finished moving to the position
            } else { fl.setPower(MathUtils.clamp((fl.getCurrentPosition()-fl.getTargetPosition() < 0 ? -1 : 1)*totalError, -1, 1));
            }if(fr.isBusy()) { fr.setPower(sForward - sStrafe - totalError); // This code looks complicated but it's simple
            } else { fr.setPower(MathUtils.clamp((fr.getCurrentPosition()-fr.getTargetPosition() < 0 ? -1 : 1)*-1*totalError,-1,1));
            }if(bl.isBusy()) { bl.setPower(-(sForward - sStrafe + totalError)); // If the motor is not finished, apply the given speed + a correction based on the angle error
            } else { bl.setPower(MathUtils.clamp((bl.getCurrentPosition()-bl.getTargetPosition() < 0 ? -1 : 1)*totalError,-1,1));
            } if(br.isBusy()) { br.setPower(sForward + sStrafe - totalError); // If the motor is finished, apply only the correction (flip flops signs because we're in RUN_TO_POSITION mode)
            } else { br.setPower(MathUtils.clamp((br.getCurrentPosition()-br.getTargetPosition() < 0 ? -1 : 1)*-1*totalError,-1,1));
            }
            pastError = error; // Move error into pastError for next loop
        }
        if(total.milliseconds() > time) {
            telemetry.addLine("time cutoff");
            telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        useEncoders(); // Switch back to normal RUN_USING_ENCODERS velocity control mode
    }
    public void gyroTurn(double turn, double sTurn, long time) { // turn is degrees
        useEncoders();
        imu.reset();
        double expectedHeading = imu.getHeading() + turn, error;
        while (expectedHeading > 180)  expectedHeading -= 360;
        while (expectedHeading <= -180) expectedHeading += 360;
        ElapsedTime e = new ElapsedTime();
        do {
            error = -1*(imu.getHeading() - expectedHeading);
            double power = (Math.abs(error) > 20) ? (Math.signum(error) * sTurn) : ((error / 20) * sTurn);
            fl.setPower(power);
            bl.setPower(power);
            fr.setPower(power);
            br.setPower(power);
        } while (Math.abs(error) > 0.01 && e.milliseconds() < time);
    }
    public void gyroTurnAbsolute(double turn, double sTurn, long time) { // turn is degrees
        useEncoders();
        imu.reset();
        double expectedHeading = turn, error;
        while (expectedHeading > 180)  expectedHeading -= 360;
        while (expectedHeading <= -180) expectedHeading += 360;
        ElapsedTime e = new ElapsedTime();
        do {
            error = -1*(imu.getHeading() - expectedHeading);
            double power = (Math.abs(error) > 20) ? (Math.signum(error) * sTurn) : ((error / 20) * sTurn);
            fl.setPower(power);
            bl.setPower(power);
            fr.setPower(power);
            br.setPower(power);
        } while (Math.abs(error) > 0.01 && e.milliseconds() < time);
    }
    public void runToPositionMode() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void useEncoders() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopAndResetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
