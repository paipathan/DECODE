package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.opencv.video.KalmanFilter;

import java.util.Objects;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;

public class Robot {

    public final Intake intake;
    public final Outtake outtake;
    public Follower follower;
    public Gamepad gamepad;

    private boolean align = false;
    private final Pose targetPosition;
    private double lastHeadingError = 0;


    public Robot(HardwareMap hwMap, Alliance alliance, Gamepad gamepad) {
        this.gamepad = gamepad;
        intake = new Intake(hwMap);
        outtake = new Outtake(hwMap);
        follower = Constants.createFollower(hwMap);
        LimeLight.init(hwMap);

        follower.update();
        follower.startTeleopDrive(true);

        targetPosition = alliance == Alliance.BLUE ? new Pose(0, 144) : new Pose(0, 144).mirror();
        configureKeyBinds();
    }

    private void configureKeyBinds() {
        Button toggleIntake = button(() -> gamepad.left_bumper)
                .whenBecomesTrue(() -> {
                    intake.start.schedule();
                })
                .whenBecomesFalse(() -> {
                    intake.stop.schedule();
                });

        Button toggleOuttake = button(() -> gamepad.right_bumper)
                .whenBecomesTrue(() -> {
                    outtake.shoot.schedule();
                })
                .whenBecomesFalse(() -> {
                    outtake.stop.schedule();
                    intake.stop.schedule();
                }).whenTrue(() -> {
                    if(outtake.getTopRPM() > 1400) {
                        intake.start.schedule();
                    }
                });

//        Button toggleAlign = button(() -> gamepad.right_bumper)
//                .whenBecomesTrue(() -> align = true)
//                .whenBecomesFalse(() -> {
//                    align = false;
//                    lastHeadingError = 0;
//                });


        Button zeroPose = button(() -> gamepad.a)
                .whenBecomesTrue(() -> follower.setPose(new Pose(0, 0, 0)));
    }



    public void periodic() {
        if(!Objects.isNull(LimeLight.getRobotPose())) {
            follower.setPose(LimeLight.getRobotPose());
        }


        if(!Outtake.isBusy) {
            outtake.topMotor.setPower(0.4);
        }

        follower.update();
        handleDrive(align);
        BindingManager.update();
        CommandManager.INSTANCE.run();
    }

    public Command followPath(PathChain path, double maxPower) {
        return new FollowPath(path, this.follower, maxPower);
    }

    private void handleDrive(boolean align) {
        if (!align) {
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
            return;
        }

        Pose currentPose = follower.getPose();
        double deltaX = targetPosition.getX() - currentPose.getX();
        double deltaY = targetPosition.getY() - currentPose.getY();
        double targetHeading = Math.atan2(deltaY, deltaX);

        double headingError = normalizeAngle(targetHeading - currentPose.getHeading());
        double headingErrorDerivative = normalizeAngle(headingError - lastHeadingError);
        lastHeadingError = headingError;

        double rotationPower = Math.max(-1, Math.min(1, (headingError * 2.0) + (headingErrorDerivative * 0.1)));

        follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, rotationPower, true);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}