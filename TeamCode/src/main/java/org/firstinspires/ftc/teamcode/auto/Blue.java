//package org.firstinspires.ftc.teamcode.auto;
//
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathBuilder;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Drawing;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.CommandManager;
//import dev.nextftc.core.commands.delays.Delay;
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.commands.utility.InstantCommand;
//
//
//@Autonomous(name="Blue Auto", group="Auto")
//public class Blue extends LinearOpMode {
//
//    public DriveTrain dt;
//    public Outtake outtake;
//    public Intake intake;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        dt = new DriveTrain(hardwareMap, gamepad1);
//        outtake = new Outtake(hardwareMap, gamepad1);
//        intake = new Intake(hardwareMap, gamepad1);
//
//        PathBuilder pathBuilder = new PathBuilder(dt.follower, Constants.pathConstraints);
//
//        PathChain firstShot = pathBuilder
//                .addPath(
//                        new BezierCurve(
//                                new Pose(56.200, 8.200),
//                                new Pose(91.239, 38.310),
//                                new Pose(65.000, 80.000)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))
//                .build();
//
//        PathChain intakePath = pathBuilder
//                .addPath(
//                        new BezierLine(new Pose(65.000, 80.000), new Pose(15.900, 83.757))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
//                .build();
//
//        dt.follower.setStartingPose(new Pose(56.2, 8.2, Math.toRadians(90)));
//        CommandManager.INSTANCE.cancelAll();
//
//        SequentialGroup autoRoutine = new SequentialGroup(
//                dt.followPath(firstShot, 1),
//                outtake.shoot,
//                new Delay(3),
//                intake.start,
//                new Delay(5),
//                outtake.stop,
//                dt.followPath(intakePath, 0.2)
//        );
//
//
//        waitForStart();
//
//        autoRoutine.schedule();
//
//        while(opModeIsActive() && !isStopRequested()) {
//            dt.follower.update();
//            CommandManager.INSTANCE.run();
//
//            Drawing.init();
//            Drawing.drawRobot(dt.follower.getPose());
//            Drawing.drawPoseHistory(dt.follower.getPoseHistory());
//            Drawing.sendPacket();
//        }
//    }
//}