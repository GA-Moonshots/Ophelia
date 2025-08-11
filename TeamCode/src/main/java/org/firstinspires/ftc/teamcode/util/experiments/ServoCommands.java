package org.firstinspires.ftc.teamcode.util.experiments;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ophelia;

public class ServoCommands extends CommandBase {
    private final Ophelia robot;
    private final ServoTest servo;
    public ServoCommands(Ophelia robot){
      this.robot = robot;
      this.servo = robot.servo;

      addRequirements(robot.servo);
    }

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        if(servo.isOpen){
            servo.close();
        }else{
            servo.open();
        }

        servo.isOpen =!servo.isOpen;

        this.isFinished();
    }

    @Override
    public boolean isFinished(){
        return true;

    }

}
