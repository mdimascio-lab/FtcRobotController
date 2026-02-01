/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

//TODO make robotOriented/fieldOriented toggle during init

@TeleOp
public class FinalTeleopFieldOriented extends OpMode {
    MecanumDrive drive = new MecanumDrive();
   // AprilTagWebcam aprilTag = new AprilTagWebcam();
    final double FEED_TIME_SECONDS = 1.0; //The feeder servos run this long when a shot is requested. previous 0.2
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 2.0; //previous 1.0

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */


    final double LAUNCHER_TARGET_VELOCITY = 4000; //1125 too fast, 1200 last, 1600 last, 1570 last, 1600 last. 2900 suggested by Abishek
    final double LAUNCHER_MIN_VELOCITY = 2000; // 1075 previous, 1200 last, 1230 last

    // Declare OpMode members.
    private DcMotorEx launcher = null;
    private DcMotorEx launcher2 = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private DcMotorEx intake = null;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive.init(hardwareMap);
       // aprilTag.init(hardwareMap, telemetry);

        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */

        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setZeroPowerBehavior(BRAKE);
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    // public void faceAprilTag(){
// TODO create method for april tag alignment


    @Override
    public void loop() {


        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            launcher2.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) { // stop flywheels
            launcher.setVelocity(STOP_SPEED);
            launcher2.setVelocity(STOP_SPEED);
        }

        if (gamepad1.right_bumper) {
            leftFeeder.setPower(-FULL_SPEED);
            rightFeeder.setPower(-FULL_SPEED);}
        else if(gamepad1.dpad_up) {
            leftFeeder.setPower(FULL_SPEED);
            rightFeeder.setPower(FULL_SPEED);}
        else {
            rightFeeder.setPower(STOP_SPEED);
            leftFeeder.setPower(STOP_SPEED);
        }

        if (gamepad1.left_bumper) {
<<<<<<< Updated upstream
<<<<<<< Updated upstream
            intake.setPower(1.0);}
=======
            intake.setVelocity(600);}
        else if (gamepad1.dpad_down) {
            intake.setVelocity(-300);}
>>>>>>> Stashed changes
=======
            intake.setVelocity(600);}
        else if (gamepad1.dpad_down) {
            intake.setVelocity(-300);}
>>>>>>> Stashed changes
        else {
            intake.setPower(0);

        }
        /*
         * Now we call our "Launch" function.
         */
        //launch(gamepad1.rightBumperWasPressed());
<<<<<<< Updated upstream

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
=======
>>>>>>> Stashed changes



        drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x); //THIS IS THE CORRECT FIELD ORIENTED LINE
        /*
         * Show the state and motor powers§
         */
        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("Heading", drive.getHeading());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


//    void launch(boolean shotRequested) {
//        switch (launchState) {
//            case IDLE:
//                if (shotRequested) {
//                    launchState = LaunchState.SPIN_UP;
//                }
//                break;
//            case SPIN_UP:
//                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
//                launcher2.setVelocity(LAUNCHER_TARGET_VELOCITY);
//                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY && launcher2.getVelocity() > LAUNCHER_MIN_VELOCITY ) {
//                    launchState = LaunchState.LAUNCH;
//                }
//                break;
//            case LAUNCH:
//                leftFeeder.setPower(-FULL_SPEED);
//                rightFeeder.setPower(-FULL_SPEED);
//                feederTimer.reset();
//                launchState = LaunchState.LAUNCHING;
//                break;
//            case LAUNCHING:
//                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
//                    launchState = LaunchState.IDLE;
//                    leftFeeder.setPower(STOP_SPEED);
//                    rightFeeder.setPower(STOP_SPEED);
//                }
//                break;
//        }
//    }
}