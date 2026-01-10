package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5453483523935) // in kg
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("leftFront") // Port 0
            .rightRearMotorName("rightRear") // Port 2
            .leftRearMotorName("leftRear") // Port 1
            .rightFrontMotorName("rightFront") // Port 3
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        ;

    public static RevHubOrientationOnRobot imuOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
    );

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(-.001991504962633513)
            .strafeTicksToInches(.001993934146752316)
            .turnTicksToInches(.004304132605789478)
            .leftPodY(3.418503937) // in inches
            .rightPodY(-3.3877952756) // in inches
            .strafePodX(-4.0962598425) // in inches
            .leftEncoder_HardwareMapName("leftFront") // Port 0
            .rightEncoder_HardwareMapName("rightFront") // Port 2
            .strafeEncoder_HardwareMapName("rightRear") // Port 3
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(imuOrientation)
    ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
            .build();
    }
}
