package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class ServoZero extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Servo servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);
        waitForStart();


        while(opModeIsActive()) {

            if(gamepad1.dpad_up) {
                servo.setPosition(-1);
            } else if (gamepad1.dpad_down) {
                servo.setPosition(1);
            }


            telemetry.addData("pos", servo.getPosition());
            updateTelemetry(telemetry);
        }
    }
}
