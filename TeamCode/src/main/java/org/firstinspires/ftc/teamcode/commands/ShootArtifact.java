package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonRobot;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.core.commands.Command;

public class ShootArtifact extends Command {
    private final AutonRobot robot;
    private final int shots;
    private final ElapsedTime timer;

    private enum State {
        START_OUTTAKE,
        WAIT_FOR_SPINUP,
        WAIT_FOR_SHOOT,
        STOP,
        DONE
    }

    private State currentState;
    private int shotsFired;

    public ShootArtifact(AutonRobot robot, int shots) {
        this.robot = robot;
        this.shots = shots;
        this.timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        setInterruptible(true);
    }

    @Override
    public void start() {
        currentState = State.START_OUTTAKE;
        shotsFired = 0;
    }

    @Override
    public void update() {
        switch (currentState) {
            case START_OUTTAKE:
                robot.outtake.start.schedule();
                currentState = State.WAIT_FOR_SPINUP;
                break;

            case WAIT_FOR_SPINUP:
                if (robot.outtake.getTopRPM() >= 950) {
                    robot.intake.start.schedule();
                    timer.reset();
                    currentState = State.WAIT_FOR_SHOOT;
                }
                break;

            case WAIT_FOR_SHOOT:
                if (robot.outtake.getTopRPM() < 850 || timer.time() > 2) {
                    robot.autoIntakeStop.schedule();
                    shotsFired++;

                    if (shotsFired >= shots) {
                        currentState = State.STOP;
                    } else {
                        currentState = State.WAIT_FOR_SPINUP;
                    }
                }
                break;

            case STOP:
                robot.autoIntakeStop.schedule();
                robot.outtake.stop.schedule();
                robot.intake.stop.schedule();
                currentState = State.DONE;
                break;

            case DONE:
                robot.outtake.stop.schedule();
                robot.intake.stop.schedule();
                break;
        }
    }

    @Override
    public boolean isDone() {
        return currentState == State.DONE;
    }

    @Override
    public void stop(boolean interrupted) {
        robot.intake.stop.schedule();
        robot.outtake.stop.schedule();
    }
}