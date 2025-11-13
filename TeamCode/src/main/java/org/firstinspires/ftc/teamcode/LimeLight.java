package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Objects;

public class LimeLight {

    private static Limelight3A ll;

    public static void init(HardwareMap hwMap) {
        ll = hwMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(100);
        ll.start();
    }

    public static boolean getRunning() {
        return ll.isRunning();
    }

    public static boolean getConnected() {
        return ll.isConnected();
    }

    public static boolean validResult() {
        return ll.getLatestResult().isValid();
    }

    public static double[] getTValues() {
        if(ll.getLatestResult().isValid()) {
            return new double[] {
                    ll.getLatestResult().getTx(),
                    ll.getLatestResult().getTy(),
                    ll.getLatestResult().getTa()
            };
        }
        return null;
    }

    public static Pose getRobotPose() {
        LLResult result = ll.getLatestResult();
        if (!result.isValid()) {
            return null;
        }

        Pose3D botPose = result.getBotpose();

        double xMeters = botPose.getPosition().x;
        double yMeters = botPose.getPosition().y;
        double yawDegrees = botPose.getOrientation().getYaw();

        double xInches = (xMeters * 39.3701);
        double yInches = (yMeters * 39.3701);
        double headingRadians = Math.toRadians(yawDegrees);

        return new Pose(
                xInches,
                yInches,
                headingRadians,
                FTCCoordinates.INSTANCE
        ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }


}
