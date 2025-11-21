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
    static double forwardV = 77.47656300615128;
    static double strafeV = 58.13652697505066;
    static double forwardZeroPowerAccel = -34.687022166256604;
    static double lateralZeroPowerAccel = -61.163609551169515;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(robotMassV1)
            .forwardZeroPowerAcceleration(forwardZeroPowerAccel)
            .lateralZeroPowerAcceleration(lateralZeroPowerAccel)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.075,
                    0,
                    0,
                    0
                    )
            )
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0,0.00001,0.6, 0.01));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(forwardV)
            .yVelocity(strafeV);



    public static TwoWheelConstants twoWheelLocalizerConstants = new TwoWheelConstants()
            .forwardPodY(-3.95)
            .strafePodX(0)
            .forwardEncoder_HardwareMapName("frontLeft")
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoder_HardwareMapName("intakeMotor")
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )
            )
            .forwardTicksToInches(0.00194283)
            .strafeTicksToInches(0.0019657756);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(twoWheelLocalizerConstants)
                .build();
    }
}
