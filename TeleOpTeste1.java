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


@TeleOp(name = "TeleOpTeste1", group = "TeleOp")
public class TeleOPTeste1" extends OpMode {

    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotor Intake;
    private DcMotor Shooter;
    private IMU imu;
    
    private boolean isShooterRunning = false; 
    private boolean wasLeftTriggerPressed = false;

     //Initializes the robot hardware, including the motors and the IMU.
     // This method is called when the Init button is pressed on the Driver station.
@Override
    public void init() {
