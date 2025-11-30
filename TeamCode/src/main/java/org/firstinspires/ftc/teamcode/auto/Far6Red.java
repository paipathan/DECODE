package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Drawing;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;

@Autonomous(name="[RED] Far 6", group="Auto")
public class Far6Red extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        Paths paths = new Paths(robot.follower);
        ElapsedTime timer  = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        robot.follower.setStartingPose(new Pose(57, 9, Math.toRadians(90)).mirror()); // might need to set starting pose heading separately
        CommandManager.INSTANCE.cancelAll();


        SequentialGroup autoRoutine = new SequentialGroup(
                robot.followPath(paths.shootPreload, 1),
                robot.shootArtifact(3),
                robot.autoIntake,
                robot.followPath(paths.alignIntake1, 0.75),
                robot.followPath(paths.intake1, 0.5),
                new Delay(1),
                robot.autoIntakeStop,
                robot.followPath(paths.shoot2, 0.5),
                robot.shootArtifact(3)
        );

        autoRoutine.schedule();
        waitForStart();

        while(opModeIsActive()) {
            robot.follower.update();
            CommandManager.INSTANCE.run();

            telemetry.addData("RPM: ", robot.outtake.getTopRPM());
            telemetry.update();

            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }

    public static class Paths {

        public PathChain shootPreload;
        public PathChain alignIntake1;
        public PathChain intake1;
        public PathChain shoot2;


        public Paths(Follower follower) {

            PathConstraints pathConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 0.9, 10, 1);

            shootPreload = follower
                    .pathBuilder().setConstraints(pathConstraints)
                    .addPath(
                            new BezierLine(new Pose(57.000, 9.000).mirror(), new Pose(61.000, 83.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133)) // change soon
                    .build();

            alignIntake1 = follower
                    .pathBuilder().setConstraints(pathConstraints)
                    .addPath(
                            new BezierLine(new Pose(61.000, 83.000).mirror(), new Pose(45.600, 83.800).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                    .build();

            intake1 = follower
                    .pathBuilder().setConstraints(pathConstraints)
                    .addPath(
                            new BezierLine(new Pose(45.600, 83.800).mirror(), new Pose(13.400, 83.800).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower
                    .pathBuilder().setConstraints(pathConstraints)
                    .addPath(
                            new BezierLine(new Pose(13.400, 83.800).mirror(), new Pose(61.000, 83.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();
        }
    }
}


