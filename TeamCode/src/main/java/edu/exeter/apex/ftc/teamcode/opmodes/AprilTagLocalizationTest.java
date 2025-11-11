package edu.exeter.apex.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import edu.exeter.apex.ftc.teamcode.subsystems.LimelightJava;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//Note, this is inefficient. Please do not use this.

@TeleOp(name = "AprilTag Vision Omni Localization", group = "Vision")
public class AprilTagLocalizationTest extends OpMode {
    private LimelightJava limelight = new LimelightJava();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Tunable control constants
    private static final double TARGET_DISTANCE_CM = 50.0;  // Target 1 meter away
    private static final double DRIVE_KP = 0.005;            // Forward/backward gain
    private static final double STRAFE_KP = 0.004;           // Left/right gain
    private static final double TURN_KP = 0.01;              // Rotation gain

    private static final double MAX_DRIVE = 0.4;
    private static final double MAX_STRAFE = 0.4;
    private static final double MAX_TURN = 0.3;

    @Override
    public void init() {
        limelight.init(hardwareMap, telemetry);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Stop motors quickly
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized. Waiting for start...");
    }

    @Override
    public void loop() {
        limelight.update();
        AprilTagDetection tag = limelight.getTagById(24); // target tag ID 24

        if (tag != null && tag.ftcPose != null) {
            double x = tag.ftcPose.x;    // Left-right offset (cm)
            double z = tag.ftcPose.z;    // Forward-back distance (cm)
            double yaw = tag.ftcPose.yaw; // Rotation (deg)

            // Compute proportional errors
            double forwardError = z - TARGET_DISTANCE_CM;
            double strafeError = -x;  // negative since +x means tag is right of camera
            double turnError = yaw;

            // Convert to power
            double forwardPower = forwardError * DRIVE_KP;
            double strafePower = strafeError * STRAFE_KP;
            double turnPower = turnError * TURN_KP;

            // Clamp outputs
            forwardPower = clamp(forwardPower, -MAX_DRIVE, MAX_DRIVE);
            strafePower = clamp(strafePower, -MAX_STRAFE, MAX_STRAFE);
            turnPower = clamp(turnPower, -MAX_TURN, MAX_TURN);

            // Apply mecanum drive power calculations
            double fl = forwardPower + strafePower - turnPower;
            double fr = forwardPower - strafePower + turnPower;
            double bl = forwardPower - strafePower - turnPower;
            double br = forwardPower + strafePower + turnPower;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // Stop once centered and at distance
            if (Math.abs(forwardError) < 5 && Math.abs(strafeError) < 3 && Math.abs(turnError) < 2) {
                stopDrive();
            }

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Distance (cm)", z);
            telemetry.addData("Strafe (x cm)", x);
            telemetry.addData("Yaw (deg)", yaw);
            telemetry.addData("Forward Pwr", forwardPower);
            telemetry.addData("Strafe Pwr", strafePower);
            telemetry.addData("Turn Pwr", turnPower);
        } else {
            stopDrive();
            telemetry.addLine("No tag detected — holding position.");
        }

    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        limelight.stop();
        stopDrive();
    }
}
