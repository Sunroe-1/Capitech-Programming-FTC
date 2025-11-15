// Imports necessary for FTC OpMode, hardware control, and IMU access.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "TeleOpInsano", group = "TeleOp")
public class TeleOPInsano extends OpMode {

    // --- Private Hardware Members ---
    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotor Intake, Shooter;
    private IMU imu;

    /**
     * Initializes the robot hardware, including the motors and the IMU.
     * This method is called when the Init button is pressed on the Driver Station.
     */
    @Override
    public void init() {
        // --- 1. Get Hardware from Configuration ---
        // Ensure these names match your robot configuration
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        // --- 2. Motor Directions (Standard Mecanum Configuration) ---
        // Assuming your robot drive system is configured such that the right side
        // must be reversed for the wheels to turn the same way.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        // Intake/Shooter directions might need adjustment
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Shooter.setDirection(DcMotor.Direction.FORWARD);

        // --- 3. Motor Run Modes ---
        // For TeleOp drive, RUN_WITHOUT_ENCODER is standard.
        // It provides direct power control (0 to 1), which is generally best for driver responsiveness.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // --- 4. IMU Initialization ---
        imu = hardwareMap.get(IMU.class, "imu");

        // Set the orientation of the control hub on the robot
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD);
        
        imu.initialize(new IMU.Parameters(RevOrientation));

        // Send telemetry to the Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Resets the IMU yaw angle to 0 degrees when the START button is pressed.
     */
    @Override
    public void start() {
        // Reset the IMU to zero degrees at the start of the match for field-centric control.
        imu.resetYaw();
    }

    /**
     * This method is called repeatedly after the user presses the Play button.
     * It handles reading joystick input and executing the field-oriented drive logic.
     */
    @Override
    public void loop() {
        // --- Joystick Reading ---
        // We use the negative of gamepad1.right_stick_y because the Y-axis on gamepads
        // is typically positive when pushing DOWN, but we want UP to be positive (forward).
        // The D-pad or a button (like A) can be used to manually reset the yaw.
        if (gamepad1.a) {
            imu.resetYaw();
            telemetry.addData("IMU", "Yaw Reset to 0");
        }

        // Right Stick for Translational Movement (Forward/Strafe)
        double axial = -gamepad1.right_stick_y; // Corrected sign for forward movement
        double lateral = gamepad1.right_stick_x;
        
        // Left Stick X for Rotation (Spin)
        double yaw = gamepad1.left_stick_x;

        // Apply a small cubic curve to translational inputs for smoother low-speed control
        // Note: For FTC, it is common to cube the power: power = power * power * power;
        axial = axial * axial * axial;
        lateral = lateral * lateral * lateral;
        // Rotation can be linear for quick response
        yaw = yaw * 0.75; // Reduce rotational sensitivity slightly

        // Execute Field-Relative Drive
        driveFieldRelative(axial, lateral, yaw);

        // Optional: Telemetry to debug values
        telemetry.addData("IMU Yaw (Deg)", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Movement", "Axial: %.2f, Lateral: %.2f, Yaw: %.2f", axial, lateral, yaw);
        telemetry.update();
    }

    /**
     * Drives the robot with the given forward, strafe, and rotate values.
     * This calculates the raw motor powers (Robot-Relative Drive).
     * @param forward The forward power of the robot, ranging from -1.0 to 1.0.
     * @param strafe The strafe power of the robot, ranging from -1.0 to 1.0.
     * @param rotate The rotate power of the robot, ranging from -1.0 to 1.0.
     */
    public void drive(double forward, double strafe, double rotate) {
        // Mecanum kinematics matrix multiplication
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // --- Power Scaling (Normalization) ---
        // This ensures no motor attempts to go above 1.0 power.
        double maxPower = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        // If the max power exceeds 1.0, scale all powers down proportionally.
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Drives the robot with the given forward, strafe, and rotate values, relative to the field.
     * This is the implementation of Field-Oriented Control (FOC).
     * @param forward The forward power (Y-axis), ranging from -1.0 to 1.0.
     * @param strafe The strafe power (X-axis), ranging from -1.0 to 1.0.
     * @param rotate The rotate power (Z-axis), ranging from -1.0 to 1.0.
     */
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        // 1. Get current robot heading (Yaw)
        // Ensure AngleUnit.RADIANS is used for trigonometric functions
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // 2. Calculate the desired angle and magnitude of the joystick vector (Field-Relative)
        // Math.atan2(y, x) gives the angle of the vector (strafe, forward)
        double inputAngle = Math.atan2(strafe, forward); 
        double magnitude = Math.hypot(forward, strafe);

        // 3. Rotate the desired movement vector by the robot's negative heading
        // This converts the Field-Relative vector into a Robot-Relative vector.
        double driveAngle = inputAngle - robotYaw;

        // 4. Calculate the new Robot-Relative X and Y components (Forward/Strafe)
        // Note: The cosine and sine are intentionally swapped here compared to standard trig,
        // because in the standard FTC coordinate system, Y (forward) is typically sine 
        // and X (strafe) is typically cosine when using atan2(y, x) with Y=forward.
        double newStrafe = magnitude * Math.cos(driveAngle);
        double newForward = magnitude * Math.sin(driveAngle);

        // 5. Feed the newly calculated Robot-Relative movement vectors into the standard drive function
        this.drive(newForward, newStrafe, rotate);
    }

    /**
     * The stop method is called when the OpMode is stopped.
     */
    @Override
    public void stop() {
        // Set all drive motors to zero power to ensure the robot stops cleanly.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
