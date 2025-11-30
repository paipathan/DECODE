package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    static double robotMassV1 = 0.938;


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(robotMassV1)
            .forwardZeroPowerAcceleration(-29.702204549353804)
            .lateralZeroPowerAcceleration(-61.74262609831347);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(73.7570735689698)
            .yVelocity(-58.13652697505066)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

//
//            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("intakeMotor")
            .forwardPodY(-3.95)
            .strafeEncoder_HardwareMapName("frontRight")
            .strafePodX(0)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.0019659201412120715)
            .strafeTicksToInches(0.001955263881874872)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )
            );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
