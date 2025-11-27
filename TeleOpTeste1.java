package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOpTeste1", group = "TeleOp")
public class TeleOpTeste1 extends OpMode {
    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotor Intake, Shooter;
    private IMU imu;

    // --- State variables for Shooter toggle ---
    // These variables act as the robot's "memory"

    private int shooterState = 0; // Remembers if the shooter should be ON or OFF
    private boolean wasLeftTriggerPressed = false; // Remembers the trigger's state from the *last loop*
    private boolean wasAButtonPressed = false; // Remembers the 'A' button's state from the *last loop*
    private boolean wasBButtonPressed = false; // Remembers the 'B' button's state from the *last loop*
    private boolean wasXButtonPressed = false; // Remembers the 'X' button's state from the *last loop*
    private boolean wasYButtonPressed = false; // Remembers the 'Y' button's state from the *last loop*
    

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

        // Define the orientation of the Rev Hub on the robot
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
        // Reset IMU yaw when starting
        imu.resetYaw();
    }

    /**
     * This method is called repeatedly when the OpMode is running.
     * This is where we will read the joystick inputs.
     */
    @Override
    public void loop() {
        // 1. Define a deadband to prevent joystick drift
        final double DEADBAND_THRESHOLD = 0.05;

        // 2. Read joystick values for Robot-Centric Movement (Right Stick)
        // Negate Y since FTC gamepads return -1.0 for stick UP (forward)
        double robotForward = gamepad1.right_stick_y;
        double robotStrafe = -gamepad1.right_stick_x;

        // 3. Read joystick value for Robot Rotation (Left Stick)
        // No negation here: Positive X (Right) -> Positive Rotate -> CW
        double robotRotate = -gamepad1.left_stick_x; 

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

        // --- 2. Shooter Logic (State Machine) ---

        // A. Get current state of all relevant buttons
        boolean isLeftTriggerPressed = gamepad1.left_trigger > 0.5;
        boolean isAButtonPressed = gamepad1.a;
        boolean isLeftBumperPressed = gamepad1.left_bumper;
        boolean isBButtonPressed = gamepad1.b;
        boolean isXButtonPressed = gamepad1.x;

        // B. Check for "Edges" (new button presses) to update the state
       if (isAButtonPressed && !wasAButtonPressed) {
    shooterState = 0; // OFF
        } 
// Press Left Trigger -> If OFF, turn ON to default SPEED_A (State 1)
        else if (isLeftTriggerPressed && !wasLeftTriggerPressed) {
            if (shooterState == 0) {
            shooterState = 1; // SPEED_A
            }
        }
// Press 'B' -> If in SPEED_A, transition to SPEED_B (State 2)
        else if (isBButtonPressed && !wasBButtonPressed) {
            if (shooterState == 1) {
            shooterState = 2; // SPEED_B
            }
        }
// Press 'X' -> If in SPEED_B, revert to SPEED_A (State 1)
        else if (isXButtonPressed && !wasXButtonPressed) {
            if (shooterState == 2) {
            shooterState = 1; // SPEED_A
            }
}
// --- Motor Power Application ---

// Left Bumper is the highest priority override (Reverse/Spit out)
if (isLeftBumperPressed) {
    Shooter.setPower(0.75);
    // Note: We don't change shooterState, so when bumper is released, it goes back to memory.
} 
// If not overriding, set power based on the stored state
else if (shooterState == 1) {
    // SPEED_A: Default shooting speed (0.8)
    Shooter.setPower(-0.75);
} 
else if (shooterState == 2) {
    // SPEED_B: Slower shooting speed (0.7)
    Shooter.setPower(-0.65);
} 
else { // shooterState == 0
    // OFF: Stop motor
    Shooter.setPower(0.0);
}

  // --- Intake and Shooter Controls ---

        // --- 1. Intake Logic (Simple If/Else If/Else) ---
        // Right Trigger (analog > 0.1) runs intake forward/in
        if (gamepad1.right_trigger > 0.1) {
            Intake.setPower(0.8);
        } 
        // Right Bumper (boolean) runs intake reverse/out
        else if (gamepad1.right_bumper) {
            Intake.setPower(-0.8);
        } 
        // Neither pressed stops the intake
        else {
            Intake.setPower(0.0);
        }

        // D. Update "previous" state variables for the *next* loop
        wasBButtonPressed = isBButtonPressed;
        wasXButtonPressed = isXButtonPressed;
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
        
        // REFINEMENT: Added telemetry for your new motors
        telemetry.addData("--- Subsystems ---", "");
        telemetry.addData("Shooter State (Memory)", wasLeftTriggerPressed ? "ON" : "OFF");
        telemetry.addData("Shooter Power", Shooter.getPower());
        telemetry.addData("Intake Power", Intake.getPower());
        
        telemetry.addData("----------", "");
        telemetry.addData("---Machine State Status---", "");
        telemetry.addData("Shooter State1", wasBButtonPressed ? "ON" : "OFF");
        telemetry.addData("Shooter State2", wasXButtonPressed ? "ON" : "OFF");

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

        if (max > 0.8) {
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
