package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.opencv.video.KalmanFilter;

import java.util.Objects;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.InstantCommand;

public class Robot {

    public final Intake intake;
    public final Outtake outtake;
    public Follower follower;
    public Gamepad gamepad;
    public Alliance alliance;

    private boolean align = false;
    private boolean robotCentric = true;
    private final Pose targetPosition;
    private double lastHeadingError = 0;

    public InstantCommand autoIntake;
    public InstantCommand autoIntakeStop;

    public Robot(HardwareMap hwMap, Alliance alliance, Gamepad gamepad) {
        this.gamepad = gamepad;
        intake = new Intake(hwMap);
        outtake = new Outtake(hwMap);
        follower = Constants.createFollower(hwMap);

        autoIntake = new InstantCommand(() -> {
            intake.start.schedule();
            outtake.bottomMotor.setPower(0.5);
        });

        autoIntakeStop = new InstantCommand(() -> {
            intake.stop.schedule();
            outtake.bottomMotor.setPower(0);
        });

        this.alliance = alliance;
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
                    outtake.bottomMotor.setPower(0.5);
                })
                .whenBecomesFalse(() -> {
                    intake.stop.schedule();
                    outtake.bottomMotor.setPower(0);
                });

        Button reverse = button(() -> gamepad.b)
                .whenBecomesTrue(() -> {
                    intake.reverse.schedule();
                    outtake.reverse.schedule();
                })
                .whenBecomesFalse(() -> {
                    intake.stop.schedule();
                    outtake.stop.schedule();
                });

        Button toggleOuttake = button(() -> gamepad.right_bumper)
                .whenBecomesTrue(() -> {
                    outtake.start.schedule();
                })
                .whenBecomesFalse(() -> {
                    outtake.stop.schedule();
                    intake.stop.schedule();
                }).whenTrue(() -> {
                    if(outtake.getTopRPM() > 1200 && !Intake.isBusy) {
                        intake.start.schedule();
                    } else if(outtake.getTopRPM() < 1200 && Intake.isBusy) {
                        intake.stop.schedule();
                    }
                });

        Button toggleAlign = button(() -> gamepad.left_trigger > 0.4)
                .whenBecomesTrue(() -> align = true)
                .whenBecomesFalse(() -> {
                    align = false;
                    lastHeadingError = 0;
                });
//hi everyome its kunsh how is everyone doing today
        Button zeroPose = button(() -> gamepad.a)
                .whenBecomesTrue(() -> follower.setPose(new Pose(0, 0, 0)));

        Button dpadUp = button(() -> gamepad.dpad_up)
                .whenTrue(() -> {
                    outtake.hood.setPosition(clamp(outtake.hood.getPosition() + 0.01, Outtake.MIN, Outtake.MAX));
                });

        Button dpadDown = button(() -> gamepad.dpad_down)
                .whenTrue(() -> {
                    outtake.hood.setPosition(clamp(outtake.hood.getPosition() - 0.01, Outtake.MIN, Outtake.MAX));
                });

    }



    public void periodic() {
        Pose limelightPose = LimeLight.getRobotPose();

        if(limelightPose != null) {
            follower.setPose(limelightPose);
        }

        if(!Outtake.isBusy) {
            outtake.topMotor.setPower(0.4);
        }

        follower.update();

        handleDrive(align);

        BindingManager.update();
        CommandManager.INSTANCE.run();
    }

    private void handleDrive(boolean align) {
        if (!align) {
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, robotCentric);
            return;
        }

        LLResult result = LimeLight.getLatestResult();

        if (result == null || !result.isValid()) {
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, robotCentric);
            return;
        }

        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, robotCentric);
            return;
        }

        int detectedId = result.getFiducialResults().get(0).getFiducialId();
        int targetId = (alliance == Alliance.BLUE) ? 20 : 24;

        if (detectedId == targetId) {
            double tx = result.getTx();
            double headingError = Math.toRadians(tx);

            double kP = 1.2;
            double rotationPower = kP * headingError;
            rotationPower = Math.max(-0.5, Math.min(0.5, rotationPower));

            follower.setTeleOpDrive(gamepad.left_stick_y, -gamepad.left_stick_x, -rotationPower, robotCentric);
        } else {
            follower.setTeleOpDrive(gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, robotCentric);
        }
    }

    public Command followPath(PathChain path, double maxPower) {
        return new FollowPath(path, this.follower, maxPower);
    }

    public Command shootArtifact(int shots) {
        return new ShootArtifact(this, shots);
    }



    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}

//zeenia is gay - zeenia p
