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

    // --- State Variables for Mechanism Control (Shooter Toggle) ---
    private boolean isShooterRunning = false; // State to track if the shooter is in the constant ON mode
    private boolean wasLeftTriggerPressed = false; // State for "edge detection" of the L-Trigger

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
        // Assuming your robot drive system is configured such that the left side
        // must be reversed for the wheels to turn the same way.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set mechanism directions (Shooter needs to run forward to shoot)
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Shooter.setDirection(DcMotor.Direction.FORWARD);

        // --- 3. Motor Run Modes ---
        // For TeleOp drive, RUN_WITHOUT_ENCODER is standard.
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
     * It handles reading joystick input and executing the field-oriented drive logic,
     * as well as controlling the Intake and Shooter.
     */
    @Override
    public void loop() {
        // ---------------------------------------------------------------------
        //                         1. DRIVE CONTROL (GAMEPAD 1)
        // ---------------------------------------------------------------------
        
        // Manual Yaw Reset (A Button)
        if (gamepad1.a) {
            imu.resetYaw();
            telemetry.addData("IMU", "Yaw Reset to 0");
        }

        // Right Stick for Translational Movement (Forward/Strafe)
        double axial = -gamepad1.right_stick_y; // Corrected sign for forward movement
        double lateral = gamepad1.right_stick_x;
        
        // Left Stick X for Rotation (Spin)
        double yaw = gamepad1.left_stick_x;

        // Apply input shaping (cubing) for smoother low-speed control
        axial = axial * axial * axial;
        lateral = lateral * lateral * lateral;
        yaw = yaw * 0.75; 

        // Execute Field-Relative Drive
        driveFieldRelative(axial, lateral, yaw);

        // ---------------------------------------------------------------------
        //                       2. MECHANISM CONTROL (GAMEPAD 2)
        // ---------------------------------------------------------------------

        // --- Intake Logic (Simple Power Control) ---
        double intakePower = 0.0;
        if (gamepad2.right_trigger > 0.1) {
            intakePower = 1.0; // Right Trigger: Intake motor runs normally (FORWARD)
        } else if (gamepad2.right_bumper) {
            intakePower = -1.0; // Right Bumper: Intake motor runs in reverse (REVERSE)
        }
        Intake.setPower(intakePower);

        // --- Shooter Logic (Toggle State Machine with Override) ---
        double shooterPower = 0.0;
        boolean isTriggerPulled = gamepad2.left_trigger > 0.1;

        // 1. Toggle ON/OFF Logic (Edge Detection on Left Trigger)
        // This toggles the constant running state only when the trigger is first pressed (rising edge).
        if (isTriggerPulled && !wasLeftTriggerPressed) {
            // Toggle the state: if running, set to stop. If stopped, set to run.
            isShooterRunning = !isShooterRunning;
        }
        wasLeftTriggerPressed = isTriggerPulled; // Update state for next loop

        // 2. State-Based Power Assignment
        if (isShooterRunning) {
            shooterPower = 1.0; // Constant running power when toggled ON
        }

        // 3. Override Conditions (Reverse / Force Stop)
        
        // Left Bumper: Force reverse
        if (gamepad2.left_bumper) {
            shooterPower = -1.0; 
            // The requirement is: "won't stop the motor but simply reverse it."
            // By overriding shooterPower here, it runs in reverse regardless of the toggle state.
            // When released, it reverts to the isShooterRunning state (constant run or off).
        }

        // A Button: Force stop the toggle state
        if (gamepad2.a) {
            isShooterRunning = false; // Clears the toggle state
            shooterPower = 0.0;       // Ensures immediate stop
        }

        Shooter.setPower(shooterPower);

        // ---------------------------------------------------------------------
        //                         3. TELEMETRY
        // ---------------------------------------------------------------------

        telemetry.addData("IMU Yaw (Deg)", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Drive Movement", "Axial: %.2f, Lateral: %.2f, Yaw: %.2f", axial, lateral, yaw);
        telemetry.addData("Intake Power", "%.2f", intakePower);
        telemetry.addData("Shooter State", isShooterRunning ? "RUNNING (Toggle ON)" : "OFF");
        telemetry.addData("Shooter Power", "%.2f", Shooter.getPower());
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
        double maxPower = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        // Scale all powers down proportionally if any motor attempts to go above 1.0.
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
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // 2. Calculate the desired angle and magnitude of the joystick vector (Field-Relative)
        double inputAngle = Math.atan2(strafe, forward); 
        double magnitude = Math.hypot(forward, strafe);

        // 3. Rotate the desired movement vector by the robot's negative heading
        double driveAngle = inputAngle - robotYaw;

        // 4. Calculate the new Robot-Relative X and Y components (Forward/Strafe)
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