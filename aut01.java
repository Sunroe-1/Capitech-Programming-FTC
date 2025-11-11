package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class aut01 extends LinearOpMode {

private DcMotor MotorC1;
private DcMotor MotorC2;
private DcMotor GarraC;


    @Override
    public void runOpMode() {

    MotorC1 = hardwareMap.get(DcMotor.class, "MotorC1");
    MotorC2 = hardwareMap.get(DcMotor.class, "MotorC2");
    GarraC = hardwareMap.get(DcMotor.class, "GarraC");
    telemetry.addData("Status", "Aguardando");
    
    double shooter = 0;
    double part = 1.0;
      
        
        if (part == 1.0);{// pura calculation pai
                          
        MotorC1.setPower(1);
        MotorC2.setPower(0.5);
        sleep(878);
        MotorC1.setPower(1);
        MotorC2.setPower(1);
        sleep(775);
        MotorC1.setPower(-1);
        MotorC2.setPower(1);
        sleep(786);
        MotorC1.setPower(1);
        MotorC2.setPower(1);
        sleep(387);
        MotorC1.setPower(-1);
        MotorC2.setPower(1);
        sleep(393);
        MotorC1.setPower(1);
        MotorC2.setPower(1);
        sleep(193);
        shooter = 1;
        MotorC1.setPower(0);
        MotorC2.setPower(0);
        sleep(1000);
        MotorC1.setPower(-1);
        MotorC2.setPower(-1);
        sleep(193);
        
                }       
    while (opModeIsActive()){

     }
    }
}
