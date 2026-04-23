package org.firstinspires.ftc.teamcode.mechanisms;

public class getRPMclass {

    public float GEAR_RATIO = 40/46;
    public double getRPM(double ticks){
        return (((ticks/28.0) * 60)*(52.0/68.0)); // on new gears 40 to 46
    }

}
