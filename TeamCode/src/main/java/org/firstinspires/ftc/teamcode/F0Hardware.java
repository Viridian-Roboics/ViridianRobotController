package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

public class F0Hardware {
    public CRServoImplEx left, right;

    public BNO055IMU imu;

    public void init(HardwareMap h) {
        left = h.get(CRServoImplEx.class, "left");
        right = h.get(CRServoImplEx.class, "right");

        left.setPwmRange(new PwmControl.PwmRange(1000,2000));
        right.setPwmRange(new PwmControl.PwmRange(1000,2000));

        left.setPwmEnable();
        right.setPwmEnable();

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setPower(0);
        right.setPower(0);

        imu = h.get(BNO055IMU.class, "imu");
    }
}
