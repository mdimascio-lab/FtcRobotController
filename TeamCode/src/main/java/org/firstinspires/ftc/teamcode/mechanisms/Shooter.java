package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    Transfer transfer = new Transfer();
    final double TARGET_SHOOTER_VELOCITY = 4000;
    final double ACCEPTABLE_MARGIN = 100;
    private DcMotorEx shooterMotorRight, shooterMotorLeft;

    public void init(HardwareMap hardwareMap) {
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "launcher");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "launcher2");

        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transfer.init(hardwareMap);
    }

    public void shootLeft() {
        shooterMotorLeft.setVelocity(TARGET_SHOOTER_VELOCITY);
        if (Math.abs(shooterMotorLeft.getVelocity()-TARGET_SHOOTER_VELOCITY) < ACCEPTABLE_MARGIN) {
            transfer.transferLeft();
        }
        shooterMotorLeft.setVelocity(0);
    }
    public void shootRight() {
        shooterMotorRight.setVelocity(TARGET_SHOOTER_VELOCITY);
        if (Math.abs(shooterMotorRight.getVelocity()-TARGET_SHOOTER_VELOCITY) < ACCEPTABLE_MARGIN) {
            transfer.transferRight();
        }
        shooterMotorRight.setVelocity(0);
    }

    public void shootAll() {
        shooterMotorRight.setVelocity(TARGET_SHOOTER_VELOCITY);
        shooterMotorLeft.setVelocity(TARGET_SHOOTER_VELOCITY);
        transfer.transferAll();
        shooterMotorRight.setVelocity(0);
        shooterMotorLeft.setVelocity(0);
    }
}
