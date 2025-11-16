package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;

@Autonomous(name="[BLUE] Far 6", group="Auto")
public class Far6Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        Paths paths = new Paths(robot.follower);

        SequentialGroup autoRoutine = new SequentialGroup(
                robot.followPath(paths.shootPreload, 1),
                robot.outtake.start,
                new Delay(2),
                robot.intake.start,
                new Delay(3),
                robot.outtake.stop,
                robot.followPath(paths.alignIntake1, 0.8),
                robot.followPath(paths.intake1, 0.25),
                robot.intake.stop,
                robot.outtake.start,
                robot.followPath(paths.shoot2, 1),
                new Delay(2),
                robot.intake.start,
                new Delay(5),
                robot.intake.stop,
                robot.outtake.stop
        );


        waitForStart();

        while(opModeIsActive()) {
            CommandManager.INSTANCE.cancelAll();
            robot.follower.setStartingPose(new Pose(57, 9));
            autoRoutine.schedule();
            robot.periodic();
        }
    }

    public static class Paths {

        public PathChain shootPreload;
        public PathChain alignIntake1;
        public PathChain intake1;
        public PathChain shoot2;

        public Paths(Follower follower) {
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.000, 9.000), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))
                    .build();

            alignIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(45.600, 83.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                    .build();

            intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.600, 83.800), new Pose(17.500, 83.800))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.500, 83.800), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();
        }
    }
}


