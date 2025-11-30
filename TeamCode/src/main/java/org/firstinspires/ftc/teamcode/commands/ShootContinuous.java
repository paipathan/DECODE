package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonRobot;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;

public class ShootContinuous extends Command {
    private final AutonRobot robot;
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

    public ShootContinuous(AutonRobot robot) {
        this.robot = robot;
        this.timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        setInterruptible(true);
    }

    @Override
    public void start() {
        currentState = State.START_OUTTAKE;
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
                    currentState = State.START_INTAKE;
                    timer.reset();
                    break;
                } else robot.autoIntake.schedule();
                break;

            case START_INTAKE:
                robot.intake.start.schedule();
                if (timer.seconds() >= 4){
                    currentState = State.STOP;
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