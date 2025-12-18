/*
Copyright 2025 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name = "AutonomousPrimeBlue", group = "Autonomous")
public class AutonomousPrimeBlue extends LinearOpMode {
    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotorEx Intake, Shooter; // CORRECT: Use DcMotorEx for velocity features
    private IMU imu;

    // 2. CONSTANTS (Ticks Per Revolution - TPR)
    private static final double SHOOTER_TPR = 28; // 20:1 UltraPlanetary
    private static final double INTAKE_TPR = 288.0;   // 72:1 Core Hex
    private static final double TARGET_SHOOTER_RPM = -3200.0;
    private static final double RPM_TOLERANCE = 50.0; // +/- 50 RPM tolerance

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        // run until the end of the match (driver presses STOP)
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
        
        Shooter.setPower(0.75);
        sleep(2000);
        Shooter.setPower(0.75);
        Intake.setPower(1);
        sleep(6000);
        Intake.setPower(0);
        Shooter.setPower(0);
        frontLeft.setPower(-0.8);
        frontRight.setPower(0.8);
        backLeft.setPower(0.8);
        backRight.setPower(-0.8);
        sleep(2300);
         frontLeft.setPower(0.8);
        frontRight.setPower(-0.8);
        backLeft.setPower(-0.8);
        backRight.setPower(0.8);
        sleep(900);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intake.setPower(0.1);

        imu.initialize(new IMU.Parameters(revOrientation));

        while (opModeIsActive()) {
            
            telemetry.addData("Status", "Running");
            telemetry.update();


    }
    }


}
