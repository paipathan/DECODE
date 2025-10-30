package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;


@Autonomous(name="BlueV2", group="Auto")
public class BlueV2 extends LinearOpMode {

    public DriveTrain dt;
    public Outtake outtake;
    public Intake intake;
    public InstantCommand shootSequence;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(hardwareMap, gamepad1);
        dt.follower.setStartingPose(new Pose(56.2, 8.2, Math.toRadians(90)));

        outtake = new Outtake(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap, gamepad1);

        PathBuilder builder = new PathBuilder(dt.follower, Constants.pathConstraints);

        PathChain firstShot = builder
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(80.82147024504084, 44.02333722287049),
                                new Pose(76, 84)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                .build();

        /*PathChain gate = builder
                .addPath(
                        new BezierCurve(
                                new Pose(48.056, 71.916),
                                new Pose(17.000, 71.916)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(90))
                .build();

        PathChain intake1 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(0,0)
                        )
                )
                .build();*/

        CommandManager.INSTANCE.cancelAll();
        SequentialGroup autoRoutine = new SequentialGroup(
                dt.followPath(firstShot),
                outtake.shoot,
                new Delay(3),
                intake.start,
                new Delay(3),
                intake.stop,
                new Delay(3),
                intake.start,
                new Delay(3),
                intake.stop,
                outtake.stop
        );


        waitForStart();
        autoRoutine.schedule();

        while(opModeIsActive() && !isStopRequested()) {
            dt.follower.update();
            CommandManager.INSTANCE.run();

            Drawing.init();
            Drawing.drawRobot(dt.follower.getPose());
            Drawing.drawPoseHistory(dt.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }
}