package org.firstinspires.ftc.teamcode.util.experiments;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ServoTest extends SubsystemBase {
    private Servo servo;     //create a class member of type Servo named servo. The Servo class comes from the FTC SDK
    public boolean isOpen = false;
    //static

    // constructor  for the servo
    public ServoTest(Ganymede ophelia){
        servo = ophelia.hardwareMap.get(Servo.class, Constants.WRIST_SERVO_NAME);
    }

    public void open() {
        servo.setPosition(1.0);  // Range is from 0.0 to 1.0
    }
    public void close() {
        servo.setPosition(0.0);  // Range is from 0.0 to 1.0
    }

}








