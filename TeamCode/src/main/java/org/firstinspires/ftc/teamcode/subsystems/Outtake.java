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
    private DcMotor topMotor;
    private DcMotor bottomMotor;
    private Servo hood;

    boolean dualMotorMode = true;

    public InstantCommand shoot2;
    public InstantCommand stop2;

    public void init(HardwareMap hwMap, Gamepad gamepad) {
        topMotor = hwMap.get(DcMotor.class, "topOuttakeMotor");
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomMotor = hwMap.get(DcMotor.class, "bottomOuttakeMotor");
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood = hwMap.get(Servo.class, "hoodServo");
        hood.setDirection(Servo.Direction.REVERSE);


        shoot2 = new InstantCommand(() -> {
            topMotor.setPower(1);
            bottomMotor.setPower(1);
        });

        stop2 = new InstantCommand(() -> {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
        });


        Button rb = button(() -> gamepad.right_bumper)
                .whenBecomesTrue(shoot2::schedule)
                .whenBecomesFalse(stop2::schedule);

    }

    public static void loop() {}
}