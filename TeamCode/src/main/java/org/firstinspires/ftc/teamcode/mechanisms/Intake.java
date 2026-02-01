package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public boolean intakeOn = false;
    private DcMotor intakeMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void toggleIntake() {
        if (intakeOn) {
            intakeOn = false;
            intakeMotor.setPower(0);
        }
        else {
            intakeOn = true;
            intakeMotor.setPower(1);
        }
    }
}
