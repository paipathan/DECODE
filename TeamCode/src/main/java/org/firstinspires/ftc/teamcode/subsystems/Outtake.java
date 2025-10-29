package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.time.Duration;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;

public class Outtake {
    private final DcMotorEx topMotor;
    private final DcMotorEx bottomMotor;

    public InstantCommand shoot;
    public InstantCommand stop;

    public double topRPM = 0;
    public double bottomRPM = 0;

    public Outtake(HardwareMap hwMap, Gamepad gamepad) {
        topMotor = hwMap.get(DcMotorEx.class, "topOuttakeMotor");
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        bottomMotor = hwMap.get(DcMotorEx.class, "bottomOuttakeMotor");
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Servo hood = hwMap.get(Servo.class, "hoodServo");
        hood.setDirection(Servo.Direction.REVERSE);


        shoot = new InstantCommand(() -> {
            topMotor.setPower(1);
            bottomMotor.setPower(-1);
            gamepad.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        });

        stop = new InstantCommand(() -> {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
            gamepad.stopRumble();
        });




        Button rb = button(() -> gamepad.right_bumper)
                .whenBecomesTrue(shoot::schedule)
                .whenBecomesFalse(stop::schedule);
    }


    public void loop() {
        topRPM = topMotor.getVelocity();
        bottomRPM = bottomMotor.getVelocity();
    }
}