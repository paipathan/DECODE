package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

@Autonomous(name="[BLUE] Far 6", group="Auto")
public class Far6Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        Paths paths = new Paths(robot.follower);

        robot.follower.setStartingPose(new Pose(57, 9, Math.toRadians(90)));
        CommandManager.INSTANCE.cancelAll();



        SequentialGroup autoRoutine = new SequentialGroup(
                robot.outtake.start,
                robot.followPath(paths.shootPreload, 1),
                new Delay(2),
                robot.intake.start,
                new Delay(5),
                robot.outtake.stop,
                robot.intake.stop,
                robot.autoIntake,
                robot.followPath(paths.alignIntake1, 0.75),
                robot.followPath(paths.intake1, 0.5),
                new Delay(1),
                robot.autoIntakeStop,
                robot.followPath(paths.shoot2, 1),
                robot.outtake.start,
                new Delay(2),
                robot.intake.start,
                new Delay(5),
                robot.outtake.stop,
                robot.intake.stop
         );

        SequentialGroup shootingTest = new SequentialGroup(
                robot.outtake.start
        );



        waitForStart();
        autoRoutine.schedule();

        while(opModeIsActive()) {
            robot.follower.update();
            CommandManager.INSTANCE.run();

            telemetry.addData("Current path chain: ", robot.follower.getCurrentPathChain());
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
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.000, 9.000), new Pose(53.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))
                    .build();

            alignIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.000, 90.000), new Pose(53.000, 83.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                    .build();

            intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.000, 83.800), new Pose(17.500, 83.800))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.500, 83.800), new Pose(53.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();
        }
    }
}


