package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDT;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;


@TeleOp(name = "bfc", group = "TeleOp")
public class BlueFC extends LinearOpMode {

    public FieldCentricDT drive;
    public Intake intake;
    public Outtake outtake;
    // public FunnelServo funnelServo\

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap, gamepad1);
        drive = new FieldCentricDT(hardwareMap, gamepad1);
        // funnelServo = new FunnelServo(hardwareMap, gamepad1);


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
