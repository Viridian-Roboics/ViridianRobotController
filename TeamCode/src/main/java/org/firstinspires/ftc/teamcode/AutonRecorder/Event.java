package org.firstinspires.ftc.teamcode.AutonRecorder;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Event {
    public double left_stick_x, left_stick_y, right_stick_x, right_stick_y, left_trigger, right_trigger;
    public boolean a, b, x, y, left_bumper, right_bumper, dpad_up, dpad_down, dpad_left, dpad_right, left_stick_button, right_stick_button;
    public Event(Gamepad g) {
        left_stick_x = g.left_stick_x;
        left_stick_y = g.left_stick_y;
        right_stick_x = g.right_stick_x;
        right_stick_y = g.right_stick_y;
        left_trigger = g.left_trigger;
        right_trigger = g.right_trigger;
        a = g.a;
        b = g.b;
        x = g.x;
        y = g.y;
        left_bumper = g.left_bumper;
        right_bumper = g.right_bumper;
        dpad_up = g.dpad_up;
        dpad_down = g.dpad_down;
        dpad_left = g.dpad_left;
        dpad_right = g.dpad_right;
        left_stick_button = g.left_stick_button;
        right_stick_button = g.right_stick_button;
    }
}
