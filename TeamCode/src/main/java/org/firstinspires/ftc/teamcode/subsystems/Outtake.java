package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.Button;

public class Outtake {
    private DcMotor motor;
    private Servo hood;

    public void init(HardwareMap hwMap, Gamepad gamepad) {
        motor = hwMap.get(DcMotor.class, "outtakeMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood = hwMap.get(Servo.class, "hoodServo");
        hood.setDirection(Servo.Direction.REVERSE);


//        Button dpUp = button(() -> gamepad.dpad_up)
//                .whenBecomesTrue(() -> hood.setPosition(hood.getPosition() + 0.1));
//
//        Button dpDown = button(() -> gamepad.dpad_down)
//                .whenBecomesTrue(() -> hood.setPosition(hood.getPosition() - 0.1));

        Button rb = button(() -> gamepad.right_bumper)
                .whenTrue(this::shoot)
                .whenBecomesFalse(this::stop);
    }



    public void shoot() {
        motor.setPower(-1);
    }

    public void stop() {
        motor.setPower(0);
    }


    public static void loop() {}
}