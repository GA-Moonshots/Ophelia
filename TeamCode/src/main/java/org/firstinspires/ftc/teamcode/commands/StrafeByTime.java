package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ophelia;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

import java.util.concurrent.TimeUnit;

public class StrafeByTime extends CommandBase {

    // REFERENCES
    private Ophelia robot;
    private Mecanum mecanum;

    // ASSETS
    private double speed;
    protected Action action;

    protected boolean finished = false;

    // TIMER
    protected Timing.Timer timer;

    // FTC DASHBOARD
    private FtcDashboard dashboard;


    public StrafeByTime(Ophelia robot, double timeout, double speed) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.dashboard = FtcDashboard.getInstance();
        this.speed = speed;

        timer = new Timing.Timer((long)(timeout * 1000), TimeUnit.MILLISECONDS);

        addRequirements(mecanum);

    }

    @Override
    public void initialize() {
        // only start the timer once the command is run and not built
        mecanum.makeRobotCentric();
        timer.start();
    }

    // The execute method keeps updating the trajectory following
    @Override
    public void execute() {
        mecanum.drive(0, speed,0);
    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {
        // Stop the drive if interrupted or completed
        mecanum.makeFieldCentric();
        mecanum.stop();
    }
}