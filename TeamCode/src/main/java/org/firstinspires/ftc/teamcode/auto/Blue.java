package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
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

        PathChain line1 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(71.888, 23.963),
                                new Pose(57.331, 84.429),
                                new Pose(30.457, 117.798)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(134))
                .build();

        dt.follower.setStartingPose(new Pose(71.88802488335925, 23.96267496111975, Math.toRadians(90)));

        CommandManager.INSTANCE.cancelAll();
        SequentialGroup autoRoutine = new SequentialGroup(
                dt.followPath(line1),
                outtake.shoot,
                new Delay(2),
                intake.start,
                new Delay(1.5),
                outtake.stop,
                new Delay(2),
                intake.stop,
                outtake.shoot,
                new Delay(2),
                intake.start,
                new Delay(2),
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