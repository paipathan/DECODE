package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
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


@Autonomous(name="Blue_goalpostauto", group="Auto")
public class Blue_goalpostauto extends LinearOpMode {

    public DriveTrain dt;
    public Outtake outtake;
    public Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap, gamepad1);

        PathBuilder builder = new PathBuilder(dt.follower, Constants.pathConstraints);

        dt.follower.setStartingPose(new Pose(27.22053675612602, 130.55775962660445, Math.toRadians(144)));

        CommandManager.INSTANCE.cancelAll();
        SequentialGroup autoRoutine = new SequentialGroup(
                outtake.shoot,
                new Delay(4),
                intake.start,
                new Delay(4),
                outtake.stop,
                new Delay(4),
                intake.stop,
                outtake.shoot,
                new Delay(4),
                intake.start,
                new Delay(4),
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