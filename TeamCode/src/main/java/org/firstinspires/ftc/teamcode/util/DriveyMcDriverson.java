package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Ophelia;


@TeleOp(name="TeleOp - Main")
public class DriveyMcDriverson extends CommandOpMode {

    @Override
    public void initialize() {
        boolean isWorking = true;
        telemetry.addData("Is it working: ", isWorking);
        telemetry.update();

        Ophelia m_robot = new Ophelia(this);
    }
}