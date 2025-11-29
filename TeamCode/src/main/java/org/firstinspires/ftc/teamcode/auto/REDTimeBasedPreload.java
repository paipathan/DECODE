package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.core.commands.CommandManager;

@Autonomous(name="[RED] Time Based Preload", group="Auto")
public class REDTimeBasedPreload extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set directions - update these to match your config
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        // Drive forward for 3 seconds
        while (opModeIsActive() && timer.seconds() < 2.5) {
            frontLeft.setPower(0.4);
            frontRight.setPower(0.4);
            backLeft.setPower(0.4);
            backRight.setPower(0.4);

            telemetry.addData("Phase", "Driving forward");
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        timer.reset();

        // Rotate left for 1 second
        while (opModeIsActive() && timer.seconds() < 0.45) {
            frontLeft.setPower(-0.4);
            frontRight.setPower(0.4);
            backLeft.setPower(-0.4);
            backRight.setPower(0.4);

            telemetry.addData("Phase", "Rotating left");
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Shoot artifact
        robot.shootArtifact(3).schedule();

        // Keep running commands until done
        while (opModeIsActive()) {
            CommandManager.INSTANCE.run();
            telemetry.addData("Phase", "Shooting");
            telemetry.update();
        }
    }
}