package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
//    private DcMotor topMotor;
    private DcMotor bottomMotor;
    private DcMotor topMotor;
    private Servo hood;

    public InstantCommand shoot;
    public InstantCommand stop;

    public Outtake(HardwareMap hwMap, Gamepad gamepad) {
        topMotor = hwMap.get(DcMotor.class, "topOuttakeMotor");
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        bottomMotor = hwMap.get(DcMotor.class, "bottomOuttakeMotor");
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood = hwMap.get(Servo.class, "hoodServo");
        hood.setDirection(Servo.Direction.REVERSE);


        shoot = new InstantCommand(() -> {
            bottomMotor.setPower(0.25);
            topMotor.setPower(1);
            gamepad.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        });

        stop = new InstantCommand(() -> {
            bottomMotor.setPower(0);
            topMotor.setPower(0);
            gamepad.stopRumble();
        });



        Button rb = button(() -> gamepad.right_bumper)
                .whenBecomesTrue(shoot::schedule)
                .whenBecomesFalse(stop::schedule);
    }


    public static void loop() {}
}