package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.Duration;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;

public class Outtake {
    private DcMotor motor;
    private Servo hood;

    public InstantCommand shoot;
    public InstantCommand stop;

    public void init(HardwareMap hwMap, Gamepad gamepad) {
        motor = hwMap.get(DcMotor.class, "outtakeMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood = hwMap.get(Servo.class, "hoodServo");
        hood.setDirection(Servo.Direction.REVERSE);

        shoot = new InstantCommand(()->motor.setPower(-1));
        stop = new InstantCommand(()->motor.setPower(0));


        Button rb = button(() -> gamepad.right_bumper)
                .whenBecomesTrue(shoot::schedule)
                .whenBecomesFalse(stop::schedule);

 
    }

    public static void loop() {}
}