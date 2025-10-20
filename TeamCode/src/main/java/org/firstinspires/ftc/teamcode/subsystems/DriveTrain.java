package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Point;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.w3c.dom.Text;

public class DriveTrain {

    public Gamepad gamepad;

    public static Follower dt;

    public void init(HardwareMap hwMap, Gamepad gamepad) {
        dt = Constants.createFollower(hwMap);
        dt.update();
        dt.setStartingPose(new Pose(0, 0, 0));
        dt.startTeleopDrive(true);

        this.gamepad = gamepad;
    }

    public void loop() {
        dt.update();
        dt.setTeleOpDrive(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, true);
    }


}
