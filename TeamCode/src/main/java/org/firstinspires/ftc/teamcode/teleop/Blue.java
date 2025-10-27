package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;


@TeleOp(name="Blue Teleop", group="TeleOp")
public class Blue extends LinearOpMode {

    public DriveTrain drive;
    public Intake intake;
    public Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap, gamepad1);
        drive = new DriveTrain(hardwareMap, gamepad1);

        drive.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        drive.startTeleopDrive();
        drive.follower.update();

        CommandManager.INSTANCE.cancelAll();


        waitForStart();




        while(opModeIsActive()) {
            BindingManager.update();
            CommandManager.INSTANCE.run();

            drive.loop();

            Drawing.init();
            Drawing.drawRobot(drive.follower.getPose());
            Drawing.drawPoseHistory(drive.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }
}
