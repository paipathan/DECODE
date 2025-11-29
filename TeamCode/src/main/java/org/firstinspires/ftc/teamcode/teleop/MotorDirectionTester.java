package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Motor Direction Tester", group="Blue")
public class MotorDirectionTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};
        String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
        boolean[] reversed = {false, false, false, false};

        int selectedMotor = 0;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastA = false;

        waitForStart();

        while (opModeIsActive()) {
            // DPad Up/Down to select motor
            if (gamepad1.dpad_up && !lastDpadUp) {
                selectedMotor = (selectedMotor + 1) % 4;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                selectedMotor = (selectedMotor - 1 + 4) % 4;
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // A to toggle direction
            if (gamepad1.a && !lastA) {
                reversed[selectedMotor] = !reversed[selectedMotor];
                motors[selectedMotor].setDirection(
                        reversed[selectedMotor] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD
                );
            }
            lastA = gamepad1.a;

            // Right trigger = run selected motor forward
            // Left trigger = run selected motor backward
            double power = gamepad1.right_trigger - gamepad1.left_trigger;

            // Stop all motors first
            for (DcMotor m : motors) m.setPower(0);

            // Run selected motor
            motors[selectedMotor].setPower(power * 0.5);

            // B = test all motors forward (to test full drivetrain)
            if (gamepad1.b) {
                for (DcMotor m : motors) m.setPower(0.3);
            }

            // X = test mecanum forward motion
            if (gamepad1.x) {
                frontLeft.setPower(0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(0.3);
                backRight.setPower(0.3);
            }

            // Y = test mecanum strafe right
            if (gamepad1.y) {
                frontLeft.setPower(0.3);
                frontRight.setPower(-0.3);
                backLeft.setPower(-0.3);
                backRight.setPower(0.3);
            }

            telemetry.addLine("=== MOTOR DIRECTION TESTER ===");
            telemetry.addLine("");
            telemetry.addLine("DPad Up/Down: Select motor");
            telemetry.addLine("A: Toggle direction");
            telemetry.addLine("RT/LT: Run motor fwd/back");
            telemetry.addLine("B: All motors forward");
            telemetry.addLine("X: Blue drive forward");
            telemetry.addLine("Y: Blue strafe right");
            telemetry.addLine("");

            for (int i = 0; i < 4; i++) {
                String arrow = (i == selectedMotor) ? ">> " : "   ";
                String dir = reversed[i] ? "REVERSE" : "FORWARD";
                telemetry.addLine(arrow + names[i] + ": " + dir);
            }

            telemetry.addLine("");
            telemetry.addLine("=== COPY THIS CONFIG ===");
            telemetry.addData("leftFront", reversed[0] ? "REVERSE" : "FORWARD");
            telemetry.addData("rightFront", reversed[1] ? "REVERSE" : "FORWARD");
            telemetry.addData("leftRear", reversed[2] ? "REVERSE" : "FORWARD");
            telemetry.addData("rightRear", reversed[3] ? "REVERSE" : "FORWARD");

            telemetry.update();
        }
    }
}