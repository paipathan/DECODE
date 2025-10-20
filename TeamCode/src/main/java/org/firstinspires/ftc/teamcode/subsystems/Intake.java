package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.bindings.Button;

public class Intake {
    private DcMotor motor;

    public void init(HardwareMap hwMap, Gamepad gamepad) {
        motor = hwMap.get(DcMotor.class, "intakeMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Button lb = button(() -> gamepad.left_bumper)
                .whenTrue(this::start)
                .whenBecomesFalse(this::stop);

        Button b = button(() -> gamepad.b)
                .whenTrue(this::reverse)
                .whenBecomesFalse(this::stop);
    }

    public void start() {
        motor.setPower(1);
    }

    public void stop() {
        motor.setPower(0);
    }

    public void reverse() {
        motor.setPower(-1);
    }
}
