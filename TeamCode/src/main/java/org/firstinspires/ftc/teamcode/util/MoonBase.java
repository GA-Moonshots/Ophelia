package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Ganymede;

public class MoonBase extends SubsystemBase{

    // COMMON INSTANCE VARIABLES
    protected Ganymede robot;
    protected Telemetry telemetry;
    public MoonBase(Ganymede robot) {
        this.robot = robot;
        this.telemetry = robot.telemetry;
    }
}
