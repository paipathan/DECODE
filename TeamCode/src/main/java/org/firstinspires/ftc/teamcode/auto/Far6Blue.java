package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;

@Autonomous(name="[BLUE] Far 6", group="Auto")
public class Far6Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Paths paths = new Paths(robot.follower);
        ElapsedTime timer  = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        robot.follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        CommandManager.INSTANCE.cancelAll();


        SequentialGroup autoRoutine = new SequentialGroup(
                // shot 1
                robot.followPath(paths.shootPreload, 1),
                robot.outtake.start,
                new Delay(3),
                robot.intake.start,
                new Delay(5),
                robot.outtake.stop,
                robot.intake.stop,

                // get ball row
                robot.autoIntake,
                robot.followPath(paths.alignIntake1, 0.75),
                robot.followPath(paths.intake1, 0.5),
                new Delay(0.75),
                robot.autoIntakeStop,

                // shot 2
                robot.followPath(paths.shoot2, 0.5),
                robot.outtake.start,
                new Delay(3),
                robot.intake.start,
                new Delay(5),
                robot.outtake.stop,
                robot.intake.stop
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
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(61.000, 83.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                    .build();

            alignIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(61.000, 83.000), new Pose(45.600, 83.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();

            intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.600, 83.800), new Pose(13.400, 83.800))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.400, 83.800), new Pose(61.000, 83.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                    .build();
        }
    }
}
