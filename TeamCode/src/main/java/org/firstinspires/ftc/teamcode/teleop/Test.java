package org.firstinspires.ftc.teamcode.teleop;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Objects;

import dev.nextftc.core.commands.CommandManager;


@TeleOp(name="Test TeleOp", group="TeleOp")
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        CommandManager.INSTANCE.cancelAll();

        waitForStart();
        while(opModeIsActive()) {
            robot.periodic();

//            telemetry.addData("[LL] Connected: ", LimeLight.getConnected());
//            telemetry.addData("[LL] Running: ", LimeLight.getRunning());
//            telemetry.addData("[LL] Valid Result", LimeLight.validResult());

            telemetry.addData("[ODO] Pose x", robot.follower.getPose().getX());
            telemetry.addData("[ODO] Pose y", robot.follower.getPose().getY());
            telemetry.addData("[ODO] Heading", robot.follower.getPose().getHeading());

            telemetry.addData("[SHOOTER] RPM", robot.outtake.topMotor.getVelocity());
            telemetry.addData("[HOOD] Tick: ", robot.outtake.hood.getPosition());
            telemetry.update();

            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }
}
