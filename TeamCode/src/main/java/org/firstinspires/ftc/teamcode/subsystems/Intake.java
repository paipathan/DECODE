package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.Mutex;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class Intake {
    private DcMotorEx motor;

    // private FunnelServo servos;
    public InstantCommand start;
    public InstantCommand stop;
    public InstantCommand reverse;

    public Command myLambdaCommand;

    public Intake(HardwareMap hwMap, Gamepad gamepad) {
        motor = hwMap.get(DcMotorEx.class, "intakeMotor");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // servos = new FunnelServo(hwMap, gamepad);

        start = new InstantCommand(()-> {
            motor.setPower(1);
            // servos.start.schedule();
        });

        stop = new InstantCommand(()->{
            motor.setPower(0);
            // servos.stop.schedule();
        });

        reverse = new InstantCommand(()-> {
            motor.setPower(-1);
            // servos.reverse.schedule();
        });


        Button lb = button(() -> gamepad.left_bumper)
                .whenBecomesTrue(start::schedule)
                .whenBecomesFalse(stop::schedule);

        Button x = button(() -> gamepad.x)
                .whenBecomesTrue(reverse::schedule)
                .whenBecomesFalse(stop::schedule);

    }

}
