package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ophelia;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class IntakeSpinByTime extends CommandBase {
    private Ophelia m_robot;
    private Intake intake;

    private Timing.Timer timer;

    private double power;

    public IntakeSpinByTime(Ophelia robot, long timeMilliseconds, double power) {
        m_robot = robot;
        intake = m_robot.intake;

        timer = new Timing.Timer(timeMilliseconds, TimeUnit.MILLISECONDS);

        this.power = power;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        intake.spinServo.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.spinServo.setPower(0.0);  //set(0);
    }
}
