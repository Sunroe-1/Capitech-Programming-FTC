package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class teleOP extends OpMode {
    
private DcMotor frontRight;
private DcMotor backRight;
private DcMotor frontLeft;
private DcMotor backLeft;
private DcMotor Intake;
private DcMotor Shooter;


int robotState = 0; 
int robotFunction = 0; 

boolean PreviousLBValue = false;
boolean PreviousRBValue = false;

boolean PreviousXbuttonValue = false;
boolean PreviousYbuttonValue = false;
boolean PreviousAButtonValue = false;
boolean PreviousBButtonValue = false;

long lastChangeTime = 0;       
long lastFunctionChangeTime = 0; 
final long debounceDelay = 300; 

    @Override
    public void init() {

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }
 
    @Override
    public void init_loop() {
    }
   
    @Override
    public void start() {
    }
    
    @Override
    public void loop() {
        double velpot = 1;
        double stop = 1 - gamepad1.right_trigger;
        double reset = 1 - gamepad1.left_trigger;
        
        boolean buttonA = gamepad1.a; 
        boolean buttonX = gamepad1.x;
        boolean buttonY = gamepad1.y;
        boolean buttonB = gamepad1.b;
        boolean LB = gamepad1.left_bumper;
        boolean RB = gamepad1.right_bumper;
        
        double y_esq = -gamepad1.left_stick_y; 
        double y_dir = -gamepad1.right_stick_y;
        double y = y_esq + y_dir;
        
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        
        double fLPower = y + x + rx;
        double bLPower = y - x + rx;
        double fRPower = y - x - rx;
        double bRPower = y + x - rx;       
        
        long now = System.currentTimeMillis();
        
        if (LB && !PreviousLBValue && (now - lastChangeTime > debounceDelay)){
             robotState += 1;
             lastChangeTime = now;
        }
        if (RB && !PreviousRBValue && (now - lastChangeTime > debounceDelay)) {
             robotState += 2;
             lastChangeTime = now;
        }
        if (robotState > 4) {
             robotState = 0;
        }
        
        robotState = robotState % 5;
        
        switch(robotState){
            case 0: velpot = 1.0; break;
            case 1: velpot = 0.75; break;
            case 2: velpot = 0.5; break;
            case 3: velpot = 0.25; break;
            case 4: velpot = 0.0; break;
        }
        double maxPower = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        fLPower /= maxPower;
        bLPower /= maxPower;
        fRPower /= maxPower;
        bRPower /= maxPower;

        fLPower *= velpot;
        bLPower *= velpot;
        fRPower *= velpot;
        bRPower *= velpot;
        
        frontLeft.setPower(fLPower);
        frontRight.setPower(fRPower);
        backLeft.setPower(bLPower);
        backRight.setPower(bRPower);        
        
        if (stop < velpot){
            velpot = 0;
        }else if (reset <= velpot) {
            robotState = 0;
        }
        if(buttonB && !PreviousBButtonValue ){ 
            robotFunction = 0;
        }
        if(buttonA && !PreviousAButtonValue){
            robotFunction = 1;
        }
        if (buttonY && !PreviousYbuttonValue) {+
            robotFunction = 4;
        }
        if (buttonX && !PreviousXbuttonValue){
                robotFunction = 2; 
        }

       
        switch (robotFunction){
            case 0: 
                Intake.setPower(0);
                Shooter.setPower(0);
                break;
            case 1:
                Intake.setPower(-1);
                Shooter.setPower(0);
                break;
                
            case 2: 
                Intake.setPower(0);
                Shooter.setPower(-0.5);
                break;
                
            case 3:
                Intake.setPower(-0.8);
                Shooter.setPower(-0.8);
                break;

            case 4: 
                Intake.setPower(0.8); 
                Shooter.setPower(0);
                break;
        }
        
        
        
        
        
        

        PreviousLBValue = LB;
        PreviousRBValue = RB;
        
        PreviousXbuttonValue = buttonX;
        PreviousYbuttonValue = buttonY;
        PreviousAButtonValue = buttonA;
        PreviousBButtonValue = buttonB;

        telemetry.addData("Potencia", "%.2f", velpot);
        telemetry.addData("Estado da potencia", robotState);
        telemetry.addData("Ato", robotFunction);
        telemetry.update();
    }
  
    @Override
    public void stop() {

        Intake.setPower(0);
        Shooter.setPower(0);
    }
}
