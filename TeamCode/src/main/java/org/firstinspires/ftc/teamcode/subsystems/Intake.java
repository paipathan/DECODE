package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;

public class Intake {
    private DcMotor intakeMotor;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    public InstantCommand start;
    public InstantCommand stop;
    public InstantCommand reverse;

    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftIntakeServo = hwMap.get(CRServo.class, "leftIntakeServo");
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo = hwMap.get(CRServo.class, "rightIntakeServo");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        start = new InstantCommand(()-> {
            intakeMotor.setPower(1);
            leftIntakeServo.setPower(1);
            rightIntakeServo.setPower(1);
        });

        stop = new InstantCommand(()-> {
            intakeMotor.setPower(0);
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);
        });

        reverse = new InstantCommand(()-> {
            intakeMotor.setPower(-1);
            leftIntakeServo.setPower(-1);
            rightIntakeServo.setPower(-1);
        });
    }
}
