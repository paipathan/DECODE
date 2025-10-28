package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.utility.InstantCommand;


public class FunnelServo {
    // private Servo leftServo;
    private Servo rightServo;
    private double leftServoPose = 0;
    private double rightServoPose = 0;

    public InstantCommand start;
    public InstantCommand stop;
    public InstantCommand reverse;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double STEP = 0.1;

    public FunnelServo(HardwareMap hwMap, Gamepad gamepad){

        // leftServo = hwMap.get(Servo.class, "leftServo");
        rightServo = hwMap.get(Servo.class, "rightServo");

        leftServoPose = 0;
        rightServoPose = 0;


        // leftServo.setPosition(leftServoPose);
        rightServo.setPosition(rightServoPose);

        start = new InstantCommand(() -> {
            adjustServos(STEP);
        });

        stop = new InstantCommand(() -> {
            // leftServo.setPosition(leftServoPose);
            rightServo.setPosition(rightServoPose);
        });

        reverse = new InstantCommand(() -> {
            adjustServos(-STEP);
        });

        Button dpad_up = button(() -> gamepad.dpad_up)
                .whenBecomesTrue(start::schedule)
                .whenBecomesFalse(stop::schedule);

        Button dpad_down = button(() -> gamepad.dpad_down)
                .whenBecomesTrue(reverse::schedule)
                .whenBecomesFalse(stop::schedule);

    }

    private void adjustServos(double step) {
        leftServoPose = clip(leftServoPose + step);
        rightServoPose = clip(rightServoPose + step);

        // leftServo.setPosition(leftServoPose);
        rightServo.setPosition(rightServoPose);
    }

    private double clip(double value) {
        return Math.max(SERVO_MIN, Math.min(SERVO_MAX, value));
    }
}
