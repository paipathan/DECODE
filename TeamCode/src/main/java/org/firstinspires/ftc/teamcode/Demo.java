package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.commands.CommandManager;

public class Demo extends LinearOpMode {


    public DcMotor motor;
    public Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.init(hardwareMap, gamepad1);

        waitForStart();

        while(opModeIsActive()) {

            intake.start.schedule();

            CommandManager.INSTANCE.run();

        }
    }
}
