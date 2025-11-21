package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;

public class Intake {
    private DcMotorEx motor;

    public InstantCommand start;
    public InstantCommand stop;
    public InstantCommand reverse;

    private final ElapsedTime timer = new ElapsedTime();
    private int lastPosition = 0;
    private double lastTime = 0;
    private final List<Double> rpmHistory = new ArrayList<>();
    private final int smoothingWindowSize = 5;

    public Intake(HardwareMap hwMap, Gamepad gamepad) {
        motor = hwMap.get(DcMotorEx.class, "intakeMotor");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Initialize RPM tracker
        lastPosition = motor.getCurrentPosition();
        lastTime = timer.seconds();

        // Commands
        start = new InstantCommand(() -> {
            motor.setPower(1);
        });

        stop = new InstantCommand(() -> {
            motor.setPower(0);
        });

        reverse = new InstantCommand(() -> {
            motor.setPower(-1);
        });

        // Button bindings
        Button lb = button(() -> gamepad.left_bumper)
                .whenBecomesTrue(start::schedule)
                .whenBecomesFalse(stop::schedule);

        Button x = button(() -> gamepad.x)
                .whenBecomesTrue(reverse::schedule)
                .whenBecomesFalse(stop::schedule);
    }

    // RPM tracking method
    public double updateAndGetSmoothedRPM() {
        double currentTime = timer.seconds();
        int currentPosition = motor.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        int deltaTicks = currentPosition - lastPosition;

        lastTime = currentTime;
        lastPosition = currentPosition;

        double ticksPerSecond = deltaTicks / deltaTime;
        double rawRPM = (ticksPerSecond * 60) / 28.0;

        rpmHistory.add(rawRPM);
        if (rpmHistory.size() > smoothingWindowSize) {
            rpmHistory.remove(0);
        }

        double sum = 0;
        for (double rpm : rpmHistory) {
            sum += rpm;
        }

        return sum / rpmHistory.size();
    }
}
// added updated and smooted rpm function
