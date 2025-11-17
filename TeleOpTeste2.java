package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOpTeste2", group = "TeleOp")
public class TeleOpTeste2 extends OpMode {
    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotor Intake, Shooter;
    private IMU imu;

    // --- State variables for Shooter toggle ---
    private boolean isShooterRunning = false;
    private boolean wasLeftTriggerPressed = false;
    private boolean wasAButtonPressed = false;

    /**
     * Initializes the robot hardware, including the motors and the IMU.
     */
    @Override
    public void init() {
        // Map motors from the hardware configuration
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        // Set motor directions (Left side reversed for standard Mecanum setup)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to run with encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior (BRAKE is generally preferred)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(revOrientation));
        
        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    /**
     * This method is called once when the Play button is pressed.
     */
    @Override
    public void start() {
        // Reset IMU yaw when starting for consistent telemetry reference
        imu.resetYaw();
    }

    /**
     * This method is called repeatedly when the OpMode is running.
     */
    @Override
    public void loop() {
        // 1. Define a deadband
        final double DEADBAND_THRESHOLD = 0.05;

        // 2. Read joystick values for Robot-Centric Movement (Right Stick)
        // Negate Y since FTC gamepads return -1.0 for stick UP (forward)
        double robotForward = -gamepad1.right_stick_y;
        double robotStrafe = gamepad1.right_stick_x;

        // 3. Read joystick value for Robot Rotation (Left Stick)
        // This is correctly set to `gamepad1.left_stick_x` for the desired CW/CCW behavior.
        // Positive X (Right) -> Positive Rotate -> Clockwise
        // Negative X (Left) -> Negative Rotate -> Counter-Clockwise
        double robotRotate = gamepad1.left_stick_x;

        // 4. Apply the deadband to all inputs
        if (Math.abs(robotForward) < DEADBAND_THRESHOLD) {
            robotForward = 0.0;
        }
        if (Math.abs(robotStrafe) < DEADBAND_THRESHOLD) {
            robotStrafe = 0.0;
        }
        if (Math.abs(robotRotate) < DEADBAND_THRESHOLD) {
            robotRotate = 0.0;
        }
        
        // 5. Call the ROBOT-CENTRIC drive function
        drive(robotForward, robotStrafe, robotRotate);

        // --- Intake and Shooter Controls ---

        // 1. Intake Logic: Right Trigger runs forward (in), Right Bumper runs reverse (out)
        if (gamepad1.right_trigger > 0.1) {
            // Intake in (assuming -1.0 is the correct direction)
            Intake.setPower(-1.0);
        } else if (gamepad1.right_bumper) {
            // Intake out
            Intake.setPower(1.0);
        } else {
            Intake.setPower(0.0);
        }

        // 2. Shooter Logic (Toggle/Override)
        boolean isLeftTriggerPressed = gamepad1.left_trigger > 0.5;
        boolean isAButtonPressed = gamepad1.a;
        boolean isLeftBumperPressed = gamepad1.left_bumper;

        // Edge detection for toggle ON (Left Trigger press)
        if (isLeftTriggerPressed && !wasLeftTriggerPressed) {
            isShooterRunning = true;
        }
        
        // Edge detection for toggle OFF ('A' Button press)
        if (isAButtonPressed && !wasAButtonPressed) {
            isShooterRunning = false;
        }

        // Set motor power based on state and overrides
        if (isLeftBumperPressed) {
            Shooter.setPower(1.0); // Override: Reverse (Spit out)
        }
        else if (isShooterRunning) {
            Shooter.setPower(-1.0); // State ON: Run forward (Shoot)
        }
        else {
            Shooter.setPower(0.0); // State OFF: Stop
        }
        
        // Update "previous" state variables for the next loop
        wasLeftTriggerPressed = isLeftTriggerPressed;
        wasAButtonPressed = isAButtonPressed;

        // --- End of Intake and Shooter Controls ---

        // 6. Add Telemetry for debugging
        telemetry.addData("--- Joysticks ---", "");
        telemetry.addData("Robot Forward (RightY)", "%.2f", robotForward);
        telemetry.addData("Robot Strafe (RightX)", "%.2f", robotStrafe);
        telemetry.addData("Robot Rotate (LeftX)", "%.2f", robotRotate);
        telemetry.addData("--- IMU ---", "");
        telemetry.addData("Robot Yaw (Degrees)", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("--- Subsystems ---", "");
        telemetry.addData("Shooter State (Memory)", isShooterRunning ? "ON" : "OFF");
        telemetry.addData("Shooter Power", Shooter.getPower());
        telemetry.addData("Intake Power", Intake.getPower());
        
        telemetry.update();
    }

    /**
     * Drives the robot with the given forward, strafe, and rotate values.
     * This function is ROBOT-CENTRIC.
     * @param forward The forward power of the robot, ranging from -1.0 to 1.0.
     * @param strafe The strafe power of the robot, ranging from -1.0 to 1.0.
     * @param rotate The rotate power of the robot, ranging from -1.0 to 1.0.
     */
    public void drive(double forward, double strafe, double rotate) {
        // Mecanum drive kinematics
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize the motor powers to keep them within the [-1.0, 1.0] range
        double max = Math.abs(frontLeftPower);
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Set the motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}