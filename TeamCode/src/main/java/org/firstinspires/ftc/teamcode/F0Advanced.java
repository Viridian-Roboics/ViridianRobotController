package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

public class F0Advanced {
    // Constants
    public static final int CPR = 8192; // counts per revolution
    public static final double wheelRadiusM = 0.01; // Wheel radius in meters
    public static final double robotWidthM = 0.2; // Robot width in meters

    // Hardware Objects
    public CRServoImplEx left, right;
    public RevIMU imu;
    public ServoEx steer;
    public Motor lEnc, rEnc; // Encoder Usage Only

    public void init(HardwareMap h) {
        // Motor setup

        left = h.get(CRServoImplEx.class, "left");
        right = h.get(CRServoImplEx.class, "right");

        left.setPwmRange(new PwmControl.PwmRange(1000,2000));
        right.setPwmRange(new PwmControl.PwmRange(1000,2000));

        left.setPwmEnable();
        right.setPwmEnable();

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPower(0);
        right.setPower(0);

        // Encoder setup

        lEnc = new Motor(h, "lEnc");
        rEnc = new Motor(h, "rEnc");

        lEnc.setInverted(true);

        lEnc.setRunMode(Motor.RunMode.RawPower);
        rEnc.setRunMode(Motor.RunMode.RawPower);

        lEnc.resetEncoder();
        rEnc.resetEncoder();

        lEnc.set(0);
        rEnc.set(0);

        // Servo setup

        steer = new SimpleServo(h, "steer", -10, 10);

        steer.turnToAngle(0);

        // Gyro setup

        imu = new RevIMU(h);

        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        p.calibrationDataFile = "BNO055IMUCalibration.json";
        p.loggingEnabled = true;
        p.loggingTag = "IMU";

        imu.init(p);
    }

    // Gets motor speed in cps
    public static double getWheelSpeed(Motor m) {
        ElapsedTime e = new ElapsedTime();
        long Pi = m.getCurrentPosition(), Pf = Pi;
        e.reset();
        while(e.milliseconds() < 20) {
            Pf = m.getCurrentPosition();
        }
        return (Pf - Pi)/e.milliseconds();
    }

    // Converts cps to m/s
    public static double mps(double cps) {
        return cps / CPR * 2 * Math.PI * wheelRadiusM;
    }

    // Returns velocity in imu units
    public double getMagVxy() {
        return Math.sqrt(Math.pow(imu.getRevIMU().getVelocity().xVeloc, 2)+Math.pow(imu.getRevIMU().getVelocity().yVeloc, 2));
    }

    // Finds angular velocity in radians/sec
    public double getAngVel() {
        ElapsedTime e = new ElapsedTime();
        double Ti = imu.getHeading(), Tf = Ti;
        e.reset();
        while(e.milliseconds() < 20) {
            Tf = imu.getHeading();
        }
        return 1000*(Tf - Ti)/e.milliseconds();

    }
}
