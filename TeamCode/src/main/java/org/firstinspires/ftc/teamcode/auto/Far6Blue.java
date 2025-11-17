package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

@Autonomous(name="[BLUE] Far 6", group="Auto")
public class Far6Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        Paths paths = new Paths(robot.follower);
        ElapsedTime timer  = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        robot.follower.setStartingPose(new Pose(57, 9, Math.toRadians(90)));
        CommandManager.INSTANCE.cancelAll();


        SequentialGroup shootOneArtifact = new SequentialGroup(
                robot.outtake.start,
                new WaitUntil(() -> robot.outtake.getTopRPM() >= 1300),
                robot.intake.start,
                new InstantCommand(timer::reset),
                new WaitUntil(() -> robot.outtake.getTopRPM() < 1000 || timer.time() > 3),
                robot.intake.stop,
                robot.outtake.stop
        );




        SequentialGroup autoRoutine = new SequentialGroup(
                robot.followPath(paths.shootPreload, 1),
                shootOneArtifact,
                shootOneArtifact,
                shootOneArtifact,
                robot.autoIntake,
                robot.followPath(paths.alignIntake1, 0.75),
                robot.followPath(paths.intake1, 0.5),
                new Delay(1),
                robot.autoIntakeStop,
                robot.followPath(paths.shoot2, 0.5),
                shootOneArtifact,
                shootOneArtifact,
                shootOneArtifact
         );

        waitForStart();
        autoRoutine.schedule();

        while(opModeIsActive()) {
            robot.follower.update();
            CommandManager.INSTANCE.run();

            telemetry.addData("Current path chain: ", robot.follower.getCurrentPathChain());
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
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.000, 9.000), new Pose(61.000, 83.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))
                    .build();

            alignIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(61.000, 83.000), new Pose(45.600, 83.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
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
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();
        }
    }
}


