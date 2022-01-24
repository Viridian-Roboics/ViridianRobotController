package org.firstinspires.ftc.teamcode.AutonRecorder;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Scanner;

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
    public Event(String str) {
        Scanner s = new Scanner(str);
        left_stick_x = s.nextDouble();
        left_stick_y = s.nextDouble();
        right_stick_x = s.nextDouble();
        right_stick_y = s.nextDouble();
        left_trigger = s.nextDouble();
        right_trigger = s.nextDouble();
        a = s.nextBoolean();
        b = s.nextBoolean();
        x = s.nextBoolean();
        y = s.nextBoolean();
        left_bumper = s.nextBoolean();
        right_bumper = s.nextBoolean();
        dpad_up = s.nextBoolean();
        dpad_down = s.nextBoolean();
        dpad_left = s.nextBoolean();
        dpad_right = s.nextBoolean();
        left_stick_button = s.nextBoolean();
        right_stick_button = s.nextBoolean();
    }
    public String toString() {
        return left_stick_x + " " + left_stick_y + " " + right_stick_x + " " + right_stick_y + " " + left_trigger + " " + right_trigger + a + " " + b + " " + x + " " + y +
               " " + left_bumper + " " + right_bumper + dpad_up + " " + dpad_down + " " + dpad_left + " " + dpad_right + " " + left_stick_button + " " + right_stick_button;
    }
}
