package org.firstinspires.ftc.teamcode.util.experiments;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ophelia;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderUpExperimental extends CommandBase {
    private Ophelia m_robot;
    private Intake m_intake;

    private Timing.Timer timer;
    private long TIMEOUT = 12;

    private int targetPosition;

    public IntakeShoulderUpExperimental(Ophelia robot, int targetPosition) {
        m_robot = robot;
        m_intake = m_robot.intake;

        this.targetPosition = targetPosition;

        timer = new Timing.Timer(TIMEOUT);

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        m_intake.shoulderMotor.setPower(0.5);
    }

    @Override
    public boolean isFinished() {
        return (m_intake.shoulderMotor.getCurrentPosition() >= (targetPosition - 5)) || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.shoulderMotor.setPower(0);
    }
}
