package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

import java.util.concurrent.TimeUnit;

public class RotateByIMU extends CommandBase {

    private final Ganymede robot;
    private final Mecanum mecanum;

    // Desired rotation speed and target heading
    private final double speed;
    private final double targetAngle;

    protected boolean finished = false;
    protected Timing.Timer timer;
    private final FtcDashboard dashboard;

    public RotateByIMU(Ganymede robot, double targetAngleOffset, double timeout, double speed) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.speed = speed;

        // Calculate the absolute target angle by adding offset to the current angle
        this.targetAngle = mecanum.getZAngle() + targetAngleOffset;

        timer = new Timing.Timer((long) timeout * 1000, TimeUnit.MILLISECONDS);
        dashboard = FtcDashboard.getInstance();
        addRequirements(mecanum);
    }

    @Override
    public void initialize() {
        mecanum.makeRobotCentric();
        timer.start();
    }

    @Override
    public void execute() {
        double currentAngle = mecanum.getZAngle();
        double angleDifference = targetAngle - currentAngle;

        // Rotate in the correct direction based on the angle difference
        double turnSpeed = Math.signum(angleDifference) * speed;
        mecanum.drive(0, 0, turnSpeed);

        // Check if rotation is within a small tolerance of the target angle
        if (Math.abs(angleDifference) < 1.0) {  // Adjust tolerance as needed
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        mecanum.makeFieldCentric();
        mecanum.stop();
    }
}
