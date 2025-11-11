package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(7.5)
        .forwardZeroPowerAcceleration(-43.552217289374944)
        .lateralZeroPowerAcceleration(-74.00923675579305)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.002, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.002, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.2, 0, 0.0005, 0.6, 0.02))
            .centripetalScaling(0.005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(60.979356440811216)
            .yVelocity(47.002489773520665);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.002984466466883962)
            .strafeTicksToInches(0.002955154122629065)
            .turnTicksToInches(.019679209316167283)
            .leftPodY(102/2.54)
            .rightPodY(-112/2.54)
            .strafePodX(128/2.54)
            .leftEncoder_HardwareMapName("front_right_drive")
            .rightEncoder_HardwareMapName("back_right_drive")
            .strafeEncoder_HardwareMapName("front_left_drive")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.2, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
