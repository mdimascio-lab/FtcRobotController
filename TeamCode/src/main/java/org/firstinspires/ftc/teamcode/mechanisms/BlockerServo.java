package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BlockerServo {
    private Servo servoPos;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servo_pos");
        servoPos.scaleRange(0.5,1.0); //set range from midpoint to 180 degrees, rather than allwoing the servo to go back to 0 degrees
        //servoPos.setDirection(Servo.Direction.REVERSE); // reverses the direction
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }
}
