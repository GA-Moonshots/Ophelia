
package org.firstinspires.ftc.teamcode.commands.lift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ophelia;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftDumpThenLevel extends CommandBase {
    // REFERENCES
    private final Ophelia robot;
    private final Lift lift;

    // ASSETS
    protected Action action;
    protected boolean finished = false;

    // TIMER
    protected Timing.Timer timer;

    // FTC DASHBOARD
    private FtcDashboard dashboard;

    // Constructor to initialize the command
    public LiftDumpThenLevel(Ophelia robot, int timeout) {
        this.robot = robot;
        lift = robot.lift;

        // default timeout
        timer = new Timing.Timer((long)timeout);
        addRequirements(lift);
    }

    // Initialize method to build the trajectory
    @Override
    public void initialize() {
        // only start the timer once the command is run and not built
        timer.start();
    }

    // The execute method keeps updating the trajectory following
    @Override
    public void execute() {
        // Create Packet for dashboard
        TelemetryPacket packet = new TelemetryPacket();

        lift.dumpBasket();
        if(timer.elapsedTime() >= 2 /* autoboxing to long */ ) {
            lift.levelBasket();
        }
    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        return timer.done();
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {

    }
}
