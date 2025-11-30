package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.core.commands.Command;

public class ShootContinuous extends Command {
    private final Robot robot;
    private final int shots;
    private final ElapsedTime timer;

    private enum State {
        START_OUTTAKE,
        WAIT,
        START_INTAKE,
        STOP,
        DONE
    }

    private State currentState;
    private int shotsFired;

    public ShootContinuous(Robot robot, int shots) {
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
                currentState = State.WAIT;
                break;

            case WAIT:
                if (robot.outtake.getTopRPM() > 950){
                    robot.intake.start.schedule();
                }

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