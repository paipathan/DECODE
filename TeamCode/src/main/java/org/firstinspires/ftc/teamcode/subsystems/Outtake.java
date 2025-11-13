package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import dev.nextftc.core.commands.utility.InstantCommand;

public class Outtake {
    public final DcMotorEx topMotor;
    public final DcMotorEx bottomMotor;

    public InstantCommand shoot;
    public InstantCommand stop;

    public static boolean isBusy = false;

    public Outtake(HardwareMap hwMap) {
        topMotor = hwMap.get(DcMotorEx.class, "topOuttakeMotor");
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        bottomMotor = hwMap.get(DcMotorEx.class, "bottomOuttakeMotor");
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Servo hood = hwMap.get(Servo.class, "hoodServo");
        hood.setDirection(Servo.Direction.REVERSE);


        shoot = new InstantCommand(() -> {
            topMotor.setPower(1);
            bottomMotor.setPower(-1);

            isBusy = true;
        });

        stop = new InstantCommand(() -> {
            topMotor.setPower(0);
            bottomMotor.setPower(0);

            isBusy = false;
        });
    }

    public double getTopRPM() {
        return topMotor.getVelocity();
    }
}