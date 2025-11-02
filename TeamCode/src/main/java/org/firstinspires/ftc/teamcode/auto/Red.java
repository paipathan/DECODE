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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;


@Autonomous(name="Red Auto", group="Auto")
public class Red extends LinearOpMode {

    public DriveTrain dt;
    public Outtake outtake;
    public Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap, gamepad1);

        PathBuilder builder = new PathBuilder(dt.follower, Constants.pathConstraints);

        PathChain line1 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(86.800, 9.200),
                                new Pose(72.560, 75.247),
                                new Pose(115.558, 120.485)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(38))
                .build();

        dt.follower.setStartingPose(new Pose(86.800, 9.2, Math.toRadians(90)));

        SequentialGroup autoRoutine = new SequentialGroup(
                dt.followPath(line1),
                outtake.shoot,
                new Delay(3),
                intake.start,
                new Delay(2.5),
                outtake.stop,
                new Delay(3),
                intake.stop,
                outtake.shoot,
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