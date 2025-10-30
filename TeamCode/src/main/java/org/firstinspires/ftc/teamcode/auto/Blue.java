package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;


@Autonomous(name="Blue Auto", group="Auto")
public class Blue extends LinearOpMode {

    public DriveTrain dt;
    public Outtake outtake;
    public Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap, gamepad1);

        PathBuilder builder = new PathBuilder(dt.follower, Constants.pathConstraints);

        dt.follower.setStartingPose(new Pose(56.2, 8.2, Math.toRadians(90)));

        PathChain firstShot = builder
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(91.239, 38.310),
                                new Pose(65, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))
                .build();

        CommandManager.INSTANCE.cancelAll();
        SequentialGroup autoRoutine = new SequentialGroup(
                dt.followPath(firstShot),
                outtake.shoot,
                new Delay(3), // revs shooter
                intake.start,
                new Delay(10),
                outtake.stop,
                intake.stop
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