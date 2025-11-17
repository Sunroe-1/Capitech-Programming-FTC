package org.firstinspires.ftc.teamcode; // REFINEMENT: Added a package declaration. OnBotJava/VSCode needs this.

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// REFINEMENT: Removed 'org.firstinspires.ftc.robotcore.external.navigation.Orientation' as it wasn't being used.

// REFINEMENT: Corrected spelling from '@TeleOP' to '@TeleOp'
@TeleOp(name = "TeleOpTeste1", group = "TeleOp")
public class TeleOPTeste1 extends OpMode {
    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotor Intake, Shooter;
    private IMU imu;

    // --- State variables for Shooter toggle ---
    // These variables act as the robot's "memory"
    private boolean isShooterRunning = false; // Remembers if the shooter should be ON or OFF
    private boolean wasLeftTriggerPressed = false; // Remembers the trigger's state from the *last loop*
    private boolean wasAButtonPressed = false; // Remembers the 'A' button's state from the *last loop*

    /**
     * Initializes the robot hardware, including the motors and the IMU.
     * This method is called when the Init button is pressed on the Driver Station.
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

        // Set motor directions
        // This is a common configuration for mecanum
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        // REFINEMENT: You might also need to reverse frontRight and backRight depending on your motor mounting.
        // E.g., frontRight.setDirection(DcMotor.Direction.REVERSE);
        // E.g., backRight.setDirection(DcMotor.Direction.REVERSE);
        // This depends on which side of the robot the motors are mounted.
        // A common setup is to reverse all motors on one side (e.g., left side).

        // Set motors to run with encoders
        // This allows for more precise speed control if you use RUN_USING_ENCODER
        // For simple setPower(), RUN_WITHOUT_ENCODER is also fine, but this is good practice.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        // REFINEMENT: Added BRAKE mode. This makes the robot stop more crisply
        // when you let go of the sticks, rather than coasting.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // REFINEMENT: Changed variable name 'RevOrientation' to 'revOrientation'
        // to follow standard Java naming conventions (variables start with lowercase).
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,  // Corrected enum value
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD // Corrected enum value
        );

        imu.initialize(new IMU.Parameters(revOrientation));
        
        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    /**
     * This method is called repeatedly after Init is pressed and before Play is pressed.
     * You can use this for things like resetting the IMU yaw.
     */
    @Override
    public void init_loop() {
        // Example: Reset yaw every time init is pressed.
        // Useful if the robot is moved after init.
        // imu.resetYaw(); 
    }

    /**
     * This method is called once when the Play button is pressed.
     */
    @Override
    public void start() {
        // You could reset the IMU yaw here as well, to set the "forward" direction
        // to wherever the robot is facing when Play is pressed.
        imu.resetYaw();
    }

    /**
     * This method is called repeatedly when the OpMode is running (after Play is pressed).
     * This is where we will read the joystick inputs.
     */
    @Override
    public void loop() {
        // --- This is the core of your request ---

        // 1. Define a deadband to prevent joystick drift
        // REFINEMENT: Added a 5% deadband. If the joystick is moved less than 5%,
        // it will be treated as 0. This stops the robot from "drifting"
        // when the sticks are centered but not perfectly at 0.0.
        final double DEADBAND_THRESHOLD = 0.05;

        // 2. Read joystick values for Field-Oriented Movement (Right Stick)
        
        // REFINEMENT: The Y-axis on FTC gamepads is inverted. "Up" is -1.0.
        // Your drive functions expect "forward" to be a positive value.
        // We must *negate* the right_stick_y value to correct this.
        double fieldForward = -gamepad1.right_stick_y;
        double fieldStrafe = gamepad1.right_stick_x;

        // 3. Read joystick value for Robot Rotation (Left Stick)
        double robotRotate = gamepad1.left_stick_x;

        // 4. Apply the deadband to all inputs
        if (Math.abs(fieldForward) < DEADBAND_THRESHOLD) {
            fieldForward = 0.0;
        }
        if (Math.abs(fieldStrafe) < DEADBAND_THRESHOLD) {
            fieldStrafe = 0.0;
        }
        if (Math.abs(robotRotate) < DEADBAND_THRESHOLD) {
            robotRotate = 0.0;
        }
        
        // REFINEMENT: Optional - "Turbo" or "Sniper" mode.
        // You can scale the inputs to make the robot slower and more precise.
        // double movementSpeedScale = 0.7; // 70% speed
        // double rotationSpeedScale = 0.5; // 50% rotation speed
        //
        // fieldForward *= movementSpeedScale;
        // fieldStrafe *= movementSpeedScale;
        // robotRotate *= rotationSpeedScale;


        // 5. Call the field-relative drive function
        // This function will take the field-relative inputs, factor in the
        // robot's current heading (from the IMU), and call the robot-centric
        // drive() function with the correct motor powers.
        driveFieldRelative(fieldForward, fieldStrafe, robotRotate);

        // --- Intake and Shooter Controls ---

        // --- 1. Intake Logic (Simple If/Else If/Else) ---
        // Right Trigger (analog > 0.1) runs intake forward
        // Right Bumper (boolean) runs intake reverse
        // Neither pressed stops the intake
        if (gamepad1.right_trigger > 0.1) {
            // Using 1.0 for full power.
            // You could also use `gamepad1.right_trigger` for variable speed intake.
            Intake.setPower(1.0);
        } else if (gamepad1.right_bumper) {
            Intake.setPower(-1.0);
        } else {
            Intake.setPower(0.0);
        }

        // --- 2. Shooter Logic (State Machine) ---

        // A. Get current state of all relevant buttons
        // We check if the trigger is pressed more than 50% (it's analog)
        boolean isLeftTriggerPressed = gamepad1.left_trigger > 0.5;
        boolean isAButtonPressed = gamepad1.a;
        boolean isLeftBumperPressed = gamepad1.left_bumper;

        // B. Check for "Edges" (new button presses) to update the state
        
        // Check for Left Trigger *press* (was not pressed, now is)
        // This is "edge detection"
        if (isLeftTriggerPressed && !wasLeftTriggerPressed) {
            isShooterRunning = true; // Turn the shooter's "memory" ON
        }
        
        // Check for 'A' Button *press* (was not pressed, now is)
        if (isAButtonPressed && !wasAButtonPressed) {
            isShooterRunning = false; // Turn the shooter's "memory" OFF
        }

        // C. Set motor power based on the *state* and *overrides*
        
        // Left Bumper is an override (reverse)
        // This check comes first, as it's the highest priority.
        if (isLeftBumperPressed) {
            Shooter.setPower(1.0);
        }
        // If not overriding, check the "memory"
        else if (isShooterRunning) {
            Shooter.setPower(-1.0); // State is ON, run forward
        }
        // If not overriding and not ON
        else {
            Shooter.setPower(0.0); // State is OFF, stop
        }
        
        // D. Update "previous" state variables for the *next* loop
        // This is critical for edge detection to work!
        wasLeftTriggerPressed = isLeftTriggerPressed;
        wasAButtonPressed = isAButtonPressed;

        // --- End of Intake and Shooter Controls ---


        // 6. Add Telemetry for debugging
        // This is crucial for seeing what the robot *thinks* it's doing.
        telemetry.addData("--- Joysticks ---", "");
        telemetry.addData("Field Forward (RightY)", "%.2f", fieldForward);
        telemetry.addData("Field Strafe (RightX)", "%.2f", fieldStrafe);
        telemetry.addData("Robot Rotate (LeftX)", "%.2f", robotRotate);
        telemetry.addData("--- IMU ---", "");
        telemetry.addData("Robot Yaw (Degrees)", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        
        // REFINEMENT: Added telemetry for your new motors
        telemetry.addData("--- Subsystems ---", "");
        telemetry.addData("Shooter State (Memory)", isShooterRunning ? "ON" : "OFF");
        telemetry.addData("Shooter Power", Shooter.getPower());
        telemetry.addData("Intake Power", Intake.getPower());
        
        telemetry.update();

        // --- Other Controls ---
        // You can add controls for your Intake and Shooter here.
        // Example:
        // REFINEMENT: Removed the old placeholder examples.
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

        // REFINEMENT: Improved normalization.
        // Your original logic was correct, but this is slightly more efficient.
        // 1. Find the largest absolute motor power
        double max = Math.abs(frontLeftPower);
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        // 2. If any motor power is > 1.0, scale all powers down
        //    proportionally.
        // This prevents one motor from "clipping" at 1.0 while others
        // are lower, which would change the robot's intended direction.
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

    /**
     * Drives the robot with the given forward, strafe, and rotate values,
     * relative to the FIELD.
     * @param forward The field-relative forward power, ranging from -1.0 to 1.0.
     * @param strafe The field-relative strafe power, ranging from -1.0 to 1.0.
     * @param rotate The robot-relative rotate power, ranging from -1.0 to 1.0.
     */
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        // Convert the (forward, strafe) vector from cartesian to polar
        // This gives us the magnitude (r) and angle (theta) of the
        // joystick input, relative to the field.
        double r = Math.hypot(strafe, forward); // Use hypot(x, y)
        double theta = Math.atan2(forward, strafe); // Use atan2(y, x)

        // Get the robot's current heading from the IMU
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // REFINEMENT: Corrected spelling from 'normalizeRadions' to 'normalizeRadians'
        // This was a critical bug that would have prevented compilation.
        
        // Rotate the joystick vector by the *negative* of the robot's heading.
        // This transforms the field-relative vector into a robot-relative vector.
        // E.g., if you push "forward" (0 rad) and robot is facing "right" (PI/2 rad),
        // the new angle will be (0 - PI/2) = -PI/2, which is "left" for the robot.
        // Pushing "left" on the robot makes it move "forward" on the field.
        double rotatedTheta = AngleUnit.normalizeRadians(theta - robotHeading);

        // Convert the new robot-relative polar vector back to cartesian
        // This gives us the new 'forward' and 'strafe' values *for the robot*.
        double newForward = r * Math.sin(rotatedTheta);
        double newStrafe = r * Math.cos(rotatedTheta);

        // Pass these new robot-centric values (and the original rotation)
        // to the robot-centric drive function.
        this.drive(newForward, newStrafe, rotate);
    }
}