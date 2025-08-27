package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.SplineToPose;
import org.firstinspires.ftc.teamcode.commands.StrafeToPose;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.SensorPackage;
import org.firstinspires.ftc.teamcode.util.experiments.AprilLimeDetect;
import org.firstinspires.ftc.teamcode.util.experiments.ServoTest;

public class Ganymede extends Robot {
    // INSTANCE VARIABLES
    public LinearOpMode opMode;
    public GamepadEx player1;
    public GamepadEx player2;
    public boolean isRed;
    public boolean left;

    // SUBSYSTEMS
    public Mecanum mecanum;
    public SensorPackage sensors;

    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public ServoTest servo;
    public Pose2d startOfTele;

    /**
     * Welcome to the Command pattern. Here we assemble the robot and kick-off the command
     *
     * @param opMode The selected operation mode
     */
    public Ganymede(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);
        // TODO : load pose from persistent pose manager
        initTele();
    }

    // OVERLOADED CONSTRUCTOR THAT RESPONDS TO AUTONOMOUS OPMODE USER QUERY
    public Ganymede(LinearOpMode opMode, boolean isRed, boolean left) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.isRed = isRed;
        this.left = left;
        initAuto();
    }

    /**
     * Set teleOp's default commands and player control bindings
     */
    public void initTele() {
        // JUST FOR TESTING
        startOfTele = new Pose2d(new Vector2d(-36, -62), Math.toRadians(180));
        mecanum = new Mecanum(this, startOfTele);
        sensors = new SensorPackage(this);
        sensors.enableAprilTagTracking();

        // Register subsystems
        // REGISTER THE SUBSYSTEM BEFORE THE DEFAULT COMMANDS
        register(mecanum, sensors);

        // Setting Default Commands. When not doing anything, respond to the controller
        mecanum.setDefaultCommand(new Drive(this));

        /*

                .__                                      ____
        ______  |  |  _____   ___.__.  ____ _______     /_   |
        \____ \ |  |  \__  \ <   |  |_/ __ \\_  __ \     |   |
        |  |_> >|  |__ / __ \_\___  |\  ___/ |  | \/     |   |
        |   __/ |____/(____  // ____| \___  >|__|        |___|
        |__|               \/ \/          \/

        */

        // BUTTON A -- TOGGLE FIELD / ROBOT CENTRIC
        Button aButtonP1 = new GamepadButton(player1, GamepadKeys.Button.A);
        aButtonP1.whenPressed(new InstantCommand(() -> {
            mecanum.toggleFieldCentric();
        }));

        // BUTTON B
        Button bButtonP1 = new GamepadButton(player1, GamepadKeys.Button.B);
        bButtonP1.whenPressed(
                new StrafeToPose(this, new Pose2d(-50, 0, 180))
        );

        // BUTTON X
        Button xButtonP1 = new GamepadButton(player1, GamepadKeys.Button.X);
        xButtonP1.whenPressed(
                new SplineToPose(this, new Pose2d(50, 0, 180))
        );

        // BUTTON Y -- Reset to Field centric
        Button yButtonP1 = new GamepadButton(player1, GamepadKeys.Button.Y);
        yButtonP1.whenPressed(
                new InstantCommand(() -> {
                    mecanum.resetFieldCentricTarget();
                })
        );

        Button dPadUpP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_UP);
        // !!! - APRIL TAG TESTING --- !!!!
        dPadUpP1.whenPressed(new AprilLimeDetect(this));

//        Button dPadDownP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_DOWN);
//        Button dPadLeftP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_LEFT);
//        Button dPadRightP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_RIGHT);

        /*

                _                                    __
               (_ )                                /'__`\
         _ _    | |    _ _  _   _    __   _ __    (_)  ) )
        ( '_`\  | |  /'_` )( ) ( ) /'__`\( '__)      /' /
        | (_) ) | | ( (_| || (_) |(  ___/| |       /' /( )
        | ,__/'(___)`\__,_)`\__, |`\____)(_)      (_____/'
        | |                ( )_| |
        (_)                `\___/'

        */

        // BUTTON B -- NOTHING
        Button bButtonP2 = new GamepadButton(player2, GamepadKeys.Button.B);

        // BUTTON Y -- Nothing
        Button yButtonP2 = new GamepadButton(player2, GamepadKeys.Button.Y);

        //  LEFT BUMPER -- NOTHING
        Button leftBumperP2 = new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER);

        // LEFT TRIGGER -- NOTHING
        Trigger leftTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);

        // DPAD DOWN -- NOTHING
        Button downDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_DOWN);

        // DPAD UP -- NOTHING
        Button upDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_UP);

        // RIGHT BUMPER -- NOTHING
        Button rightBumperP2 = new GamepadButton(player2, GamepadKeys.Button.RIGHT_BUMPER);

        // RIGHT TRIGGER -- NOTHING
        Trigger rightTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);

        // JOYSTICK LEFT - NOTHING
        Button joyStickLeftTrigger = new GamepadButton(player2, GamepadKeys.Button.LEFT_STICK_BUTTON);

        // JOYSTICK RIGHT - NOTHING
        Button joyStickRightTrigger = new GamepadButton(player2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
    }

    /**
     * -----------------
     * ------ AUTO -----
     */
    public void initAuto() {
        Pose2d start;
        if (left && isRed) {
            start = new Pose2d(new Vector2d(-36, -62), Math.toRadians(180)); // starting position for red left
        } else {
            start = new Pose2d(new Vector2d(36, 62), Math.toRadians(0));
        }

        mecanum = new Mecanum(this, start);
        mecanum.makeFieldCentric();
        sensors = new SensorPackage(this);
        sensors.enableAprilTagTracking();

        register(mecanum, sensors);

        new SequentialCommandGroup(
                new StrafeToPose(this, new Pose2d(-50, 0, 180))
        ).schedule();
    }
}
