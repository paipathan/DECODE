package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class Intake {
    private DcMotor motor;

    public InstantCommand start;
    public InstantCommand stop;
    public InstantCommand reverse;

    public Command myLambdaCommand;

    public Intake(HardwareMap hwMap, Gamepad gamepad) {
        motor = hwMap.get(DcMotor.class, "intakeMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        start = new InstantCommand(()->motor.setPower(-1));
        stop = new InstantCommand(()->motor.setPower(0));
        reverse = new InstantCommand(()->motor.setPower(1));


        Button lb = button(() -> gamepad.left_bumper)
                .whenBecomesTrue(start::schedule)
                .whenBecomesFalse(stop::schedule);

    }

}
