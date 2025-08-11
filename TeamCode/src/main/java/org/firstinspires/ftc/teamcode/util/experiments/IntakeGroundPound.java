package org.firstinspires.ftc.teamcode.util.experiments;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ophelia;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class IntakeGroundPound extends CommandBase {
    private final Ophelia robot;
    private final Intake intake;

    private final double TIMEOUT = 2.0; // seconds
    private final Timing.Timer timer;

    public IntakeGroundPound(Ophelia robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.timer = new Timing.Timer((long) TIMEOUT * 1000, TimeUnit.MILLISECONDS); // Convert to milliseconds

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
        intake.shoulderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void execute() {
        // TODO: see if positive or negative 0.1
        intake.shoulderMotor.setPower(0.1);
        // sigma protection
        if (intake.shoulderMotor.isOverCurrent()){
            intake.shoulderMotor.setPower(0);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopShoulder();
        if (!interrupted) {
            intake.shoulderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        intake.shoulderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
