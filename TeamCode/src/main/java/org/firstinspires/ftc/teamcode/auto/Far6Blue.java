package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="[BLUE] Far 6", group="Auto")
public class Far6Blue extends OpMode {

    private Robot robot;
    private Follower follower;
    private ElapsedTime pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(61, 83, Math.toRadians(133));

    private PathChain shootPreload;

    public void buildPaths() {
        shootPreload = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(shootPreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // Shoot preload here
                    setPathState(-1); // Done
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        follower = robot.follower;
        pathTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        pathTimer.reset();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("RPM", robot.outtake.getTopTPS());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        Drawing.init();
        Drawing.drawRobot(follower.getPose());
        Drawing.drawPoseHistory(follower.getPoseHistory());
        Drawing.sendPacket();
    }

    @Override
    public void stop() {}
}