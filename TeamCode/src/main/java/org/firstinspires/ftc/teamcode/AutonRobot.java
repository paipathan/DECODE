package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.commands.ShootContinuous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.opencv.video.KalmanFilter;

import java.util.Objects;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.InstantCommand;

public class AutonRobot {

    public final Intake intake;
    public final Outtake outtake;
    public Follower follower;

    public InstantCommand autoIntake;
    public InstantCommand autoIntakeStop;

    public AutonRobot(HardwareMap hwMap) {
        intake = new Intake(hwMap);
        outtake = new Outtake(hwMap);
        follower = Constants.createFollower(hwMap);
        follower.update();

        autoIntake = new InstantCommand(() -> {
            intake.start.schedule();
            outtake.bottomMotor.setPower(0.5);
        });

        autoIntakeStop = new InstantCommand(() -> {
            intake.stop.schedule();
            outtake.bottomMotor.setPower(0);
        });
    }

    public Command followPath(PathChain path, double maxPower) {
        return new FollowPath(path, this.follower, maxPower);
    }

    public Command shootArtifact(int shots) {
        return new ShootArtifact(this, shots);
    }

    public Command shootContinuous(){
        return new ShootContinuous(this);
    }
}