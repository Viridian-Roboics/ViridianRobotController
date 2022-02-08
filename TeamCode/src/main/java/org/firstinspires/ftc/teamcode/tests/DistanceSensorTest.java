package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class DistanceSensorTest extends OpMode {
    DistanceSensor d;
    @Override
    public void init() {
        d = hardwareMap.get(DistanceSensor.class, "d");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance (mm)",d.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}
