package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

@TeleOp
public class NewTeleop extends OpMode {
    private boolean intakeTogglePressed = false;
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    Transfer transfer = new Transfer();
    MecanumDrive drive = new MecanumDrive();

    @Override
    public void init() {
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        transfer.init(hardwareMap);
        drive.init(hardwareMap);
    }
    public void loop() {
        drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

        if (gamepad1.a && !intakeTogglePressed) {
            intake.toggleIntake();
            intakeTogglePressed= true;
        }
        if (!gamepad1.a) {
            intakeTogglePressed=false;
        }
        if (gamepad1.b) {shooter.shootAll();}
        if (gamepad1.x) {transfer.transferAll();}
}}
