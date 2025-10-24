package org.firstinspires.ftc.teamcode.pedroPathing;

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
    static double forwardV = 77.47656300615128; // test 2: 79.07251679918099
    static double strafeV = 58.13652697505066; // test 2: 59.71897433734531 || 57.41994932375924
    static double forwardZeroPowerAccel = -34.687022166256604; // test 2: -34.687022166256604 || -33.74762702215598 || abhays test: -26.917226893087108
    static double lateralZeroPowerAccel = -61.163609551169515; // test 2: -63.878670347567514 || -61.163609551169515

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(robotMassV1)
            .forwardZeroPowerAcceleration(forwardZeroPowerAccel)
            .lateralZeroPowerAcceleration(lateralZeroPowerAccel)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.075, // good range is 0.75 to 0.85 cant rlly know yet till we try actual pathing
                    0,
                    0.11, // 0.01 to 0.12
                    0.023 // good range is 0.21 to 0.28.
                    )
            )
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01));

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
            .yVelocity(strafeV); // test 2: 59.71897433734531 || 57.41994932375924



    public static TwoWheelConstants twoWheelLocalizerConstants = new TwoWheelConstants()
            .forwardPodY(-3.95)
            .strafePodX(0)
            .forwardEncoder_HardwareMapName("intakeMotor")
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoder_HardwareMapName("frontLeft")
            .strafeEncoderDirection(Encoder.REVERSE)
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
