package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class MecanumDrive extends LinearOpMode {
    // Motors
    DcMotor frontRight = null;
    DcMotor frontLeft = null;
    DcMotor backRight = null;
    DcMotor backLeft = null;
    DcMotor intakemotor = null;
    DcMotor outtake = null;

    // RPM Tracker
    ShooterRPMTracker rpmTracker;
    List<Double> fullPowerRPMLog = new ArrayList<>();

    @Override
    public void runOpMode() {
        // Map motors
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        outtake = hardwareMap.get(DcMotor.class, "outtake");

        // Set directions
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        intakemotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Encoder mode for shooter
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize RPM tracker
        rpmTracker = new ShooterRPMTracker(outtake);

        waitForStart();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double intakePower = gamepad1.left_trigger;
            double shooterPower = gamepad1.right_trigger > 0.1 ? 1.0 : 0.0;

            double frontRightPower = drive + strafe - turn;
            double frontLeftPower = drive - strafe + turn;
            double backRightPower = drive - strafe - turn;
            double backLeftPower = drive + strafe + turn;

            frontRight.setPower(frontRightPower * 0.75);
            frontLeft.setPower(frontLeftPower * 0.75);
            backRight.setPower(backRightPower * 0.75);
            backLeft.setPower(backLeftPower * 0.75);

            intakemotor.setPower(intakePower);
            outtake.setPower(shooterPower);

            double currentRPM = rpmTracker.updateAndGetRPM();
            telemetry.addData("Shooter RPM", currentRPM);
            telemetry.update();

            if (shooterPower >= 1.0) {
                fullPowerRPMLog.add(currentRPM);
            }
        }
    }

    
    public static class ShooterRPMTracker {
        private final DcMotor shooterMotor;
        private final ElapsedTime timer = new ElapsedTime();
        private int lastPosition = 0;
        private double lastTime = 0;

        public ShooterRPMTracker(DcMotor shooterMotor) {
            this.shooterMotor = shooterMotor;
            this.lastPosition = shooterMotor.getCurrentPosition();
            this.lastTime = timer.seconds();
        }

        public double updateAndGetRPM() {
            double currentTime = timer.seconds();
            int currentPosition = shooterMotor.getCurrentPosition();

            double deltaTime = currentTime - lastTime;
            int deltaTicks = currentPosition - lastPosition;

            lastTime = currentTime;
            lastPosition = currentPosition;

            double ticksPerSecond = deltaTicks / deltaTime;
            return (ticksPerSecond * 60) / 28.0; // 28 is just the enocder thingy
        }
    }
}
