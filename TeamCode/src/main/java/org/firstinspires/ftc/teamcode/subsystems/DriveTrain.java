package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class DriveTrain {

    public Gamepad gamepad;
    public Follower follower;
    private boolean alignToGoal = false;

    private Pose targetPosition = new Pose(0, 144);

    private double kP = 2.0;
    private double kD = 0.1;

    private double lastHeadingError = 0;

    public DriveTrain(HardwareMap hwMap, Gamepad gamepad) {
        follower = Constants.createFollower(hwMap);
        this.gamepad = gamepad;

        Button lt = button(() -> gamepad.a)
                .whenBecomesTrue(() -> alignToGoal = true)
                .whenBecomesFalse(() -> {
                    alignToGoal = false;
                    lastHeadingError = 0;
                });

        Button a = button(() -> gamepad.a)
                .whenBecomesTrue(() -> {
                    follower.setPose(new Pose(0,0,Math.toRadians(90)));
                    follower.update();
                });

    }

    public Command followPath(PathChain path) {
        return new FollowPath(path, this);
    }


    public void loop() {
        follower.update();

        if (alignToGoal) {
            driveWithHeadingLock();
        } else {
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
        }
    }


    private void driveWithHeadingLock() {
        Pose currentPose = follower.getPose();

        double deltaX = targetPosition.getX() - currentPose.getX();
        double deltaY = targetPosition.getY() - currentPose.getY();
        double targetHeading = Math.atan2(deltaY, deltaX);

        double headingError = targetHeading - currentPose.getHeading();

        headingError = normalizeAngle(headingError);


        double headingErrorDerivative = 0;
        if (lastHeadingError != 0 || alignToGoal) {
            headingErrorDerivative = headingError - lastHeadingError;
            headingErrorDerivative = normalizeAngle(headingErrorDerivative);
        }
        lastHeadingError = headingError;

        double rotationPower = (headingError * kP) + (headingErrorDerivative * kD);

        rotationPower = Math.max(-1, Math.min(1, rotationPower));

        follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, rotationPower, true);
    }


    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive(true);
    }

    public void setAlignmentTarget(double x, double y) {
        targetPosition = new Pose(x, y);
    }
    public void setAlignmentTarget(Pose target) {
        targetPosition = target;
    }
    public void setAlignmentPID(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

}


class FollowPath extends Command {

    private final PathChain path;
    private final DriveTrain driveTrain;

    public FollowPath(PathChain path, DriveTrain driveTrain) {
        setInterruptible(true);
        this.path = path;
        this.driveTrain = driveTrain;
    }

    @Override
    public void start() {
        driveTrain.follower.followPath(path, false);
    }

    @Override
    public void update() {
        driveTrain.follower.update();
    }

    @Override
    public boolean isDone() {
        return driveTrain.follower.atParametricEnd();
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted) {
            driveTrain.follower.breakFollowing();
        }
    }
}