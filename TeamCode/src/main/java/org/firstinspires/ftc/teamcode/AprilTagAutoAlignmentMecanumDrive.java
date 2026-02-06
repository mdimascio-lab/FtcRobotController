//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
//import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//@TeleOp
//public class AprilTagAutoAlignmentMecanumDrive extends OpMode {
//
//    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
//    private final MecanumDrive drive = new MecanumDrive();
//
//  // ----------------- PD controller ---------------------
//
//    double kp = 0.002;
//    double error = 0;
//    double lastError = 0;
//    double goalX = 0; //offset here
//    double angleTolerance = 0.4;
//    double kD = 0.0001;
//    double curTime = 0;
//    double lastTime = 0;
//
//    // --------------- driving setup ----------------------
//    double forward, strafe, rotate;
//
//    //--------------- controller based PD tuning -----------------
//    double[] stepSizes = {1.0, 0.1, 0.001, 0.0001};
//    int stepIndex = 2;
//
//    @Override
//    public void init() {
//        aprilTagWebcam.init(hardwareMap, telemetry);
//        drive.init(hardwareMap, false);
//
//        telemetry.addLine("Initialized");
//    }
//
//    public void start(){
//        resetRuntime();
//        curTime = getRuntime();
//
//    }
//    @Override
//    public void loop() {
//        //--------------- get mecanum drive inputs -----------------
//        forward = -gamepad1.left_stick_y;
//        strafe = gamepad1. left_stick_x;
//        rotate = gamepad1.right_stick_x;
//        //--------------- get april Tag info -----------------
//        aprilTagWebcam.update();
//        AprilTagDetection id = aprilTagWebcam.getTagBySpecificId(20); // TODO CHECK WHAT THE ACTUAL ID IS
//        //--------------- get april Tag info -----------------
//        if (gamepad1.left_trigger > 0.3) {
//
//
//        }
//        }
//
//
//
//
//
//    }
//
//
