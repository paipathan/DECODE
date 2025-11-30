package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.core.commands.CommandManager;


@TeleOp(name="[BLUE] TeleOp", group="TeleOp")
public class Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        CommandManager.INSTANCE.cancelAll();

        waitForStart();
        while(opModeIsActive()) {
            robot.periodic();

            telemetry.addData("[SHOOTER] RPM", robot.outtake.getTopRPM());
            telemetry.update();

            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }
}