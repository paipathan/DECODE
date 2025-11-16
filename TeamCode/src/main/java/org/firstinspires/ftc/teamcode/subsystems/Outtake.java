package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;

public class Outtake {
    public final DcMotorEx topMotor;
    public final DcMotorEx bottomMotor;

    public InstantCommand start;
    public InstantCommand stop;

    public InstantCommand autoStart;

    public final static double MIN = 0.3006;
    public final static double MAX = 1;

    public static boolean isBusy = false;

    public static InterpLUT lut = new InterpLUT();

    public Servo hood;

    public Outtake(HardwareMap hwMap) {
        topMotor = hwMap.get(DcMotorEx.class, "topOuttakeMotor");
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        bottomMotor = hwMap.get(DcMotorEx.class, "bottomOuttakeMotor");
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood = hwMap.get(Servo.class, "hoodServo");
        hood.setDirection(Servo.Direction.REVERSE);

        start = new InstantCommand(() -> {
            topMotor.setPower(1);
            bottomMotor.setPower(-1);

            isBusy = true;
        });


        stop = new InstantCommand(() -> {
            topMotor.setPower(0);
            bottomMotor.setPower(0);

            isBusy = false;
        });

        configureLUT();
    }

    public double getTopRPM() {
        return topMotor.getVelocity();
    }

    private void configureLUT() {

    }
}