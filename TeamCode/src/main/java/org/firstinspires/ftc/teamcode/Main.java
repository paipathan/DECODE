package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name="Main", group="Main")
public class Main extends LinearOpMode {

    public Follower robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = Constants.createFollower(hardwareMap);
        robot.update();
        robot.setStartingPose(new Pose(0, 0, 0));
        robot.startTeleopDrive(true);
        waitForStart();

        while(opModeIsActive()) {
            robot.update();
            robot.setTeleOpDrive(-gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true);
        }
    }

}
