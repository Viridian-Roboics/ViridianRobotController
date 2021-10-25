package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

public class F0HardwareSteeringWithGyro {
    public CRServoImplEx left, right;
    public ServoEx steer;
    public RevIMU imu;


    public void init(HardwareMap h) {
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

        // Servo setup

        steer = new SimpleServo(h, "steer", -10, 10);

        steer.turnToAngle(0);

        // IMU

        imu = new RevIMU(h, "imu");
        imu.init();
    }
}
