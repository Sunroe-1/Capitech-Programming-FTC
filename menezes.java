
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp

public class menezes extends OpMode {
    /* Declare OpMode members. */
    
private DcMotor MotorC1;
private DcMotor MotorC2;
private DcMotor GarraC;
  
    @Override
    public void init() {
        MotorC1 = hardwareMap.get(DcMotor.class, "MotorC1");
        MotorC2 = hardwareMap.get(DcMotor.class, "MotorC2");
        GarraC = hardwareMap.get(DcMotor.class, "GarraC");
        telemetry.addData("Status", "Aguardando");
        MotorC2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
     *Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
     
    @Override
    public void init_loop() {
        
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double velpot = 1;
        double trigger = 1 - gamepad1.right_trigger;
    
        if (trigger < velpot) {
            velpot = 1;
        }else if (gamepad1.left_bumper){
            velpot = 0.75;
        }
        
        if (gamepad1.left_bumper)  {
            velpot = 0.5;
        }else if (gamepad1.right_bumper){
            velpot = 0.20;
        }
        
     
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x;
        
        double c1Power = (y + x) * velpot;
        double c2Power = (y - x) * velpot;
        
        double claw = gamepad1.left_stick_y;
        
        MotorC1.setPower(c1Power);
        MotorC2.setPower(c2Power);
        
        GarraC.setPower(claw);
        
        telemetry.addData("Escala de Potencia", "%.2f", velpot);
        telemetry.update();
        
                  
          
   
    }
  

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        MotorC1.setPower(0);
        MotorC2.setPower(0);
        GarraC.setPower(0);
    }
}
