
package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ophelia;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class IntakeShoulderUp extends CommandBase {
    private final Ophelia robot;
    private final Intake intake;
     // acceptable error in ticks
    private final double TIMEOUT = 3.0; // seconds
    protected Timing.Timer timer;

    public IntakeShoulderUp(Ophelia robot) {
        this.robot = robot;
        this.intake = this.robot.intake;


        timer = new Timing.Timer((long) TIMEOUT * 1000, TimeUnit.MILLISECONDS);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        double power;

        if (timer.elapsedTime() < 1500) {
            intake.shoulderMotor.setPower(0.75); // First 1500ms
        } else if (timer.elapsedTime() < 2000 && timer.elapsedTime() >= 1500) {
            intake.shoulderMotor.setPower(0.2); // Next 500ms (1500-2000ms)
        } else {
            intake.shoulderMotor.setPower(0);
        }


    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.shoulderMotor.setPower(0.0);
    }
}