import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class IMUteste extends LinearOpMode {


    private IMU imu;
    private DcMotor MotorC1;
    private DcMotor MotorC2;
    private DcMotor GarraC;
    

    private double getYaw() {
        
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void turnToAngle(double targetAngle, double potencia) {
        double currentAngle;
        double angleDifference;
        double tolerance = 1.0; 

        currentAngle = getYaw();
        angleDifference = targetAngle - currentAngle;

     
        while (opModeIsActive() && Math.abs(angleDifference) > tolerance) {
            
            angleDifference = targetAngle - currentAngle;
            while (angleDifference > 180) angleDifference -= 360;
            while (angleDifference < -180) angleDifference += 360;

          
            if (angleDifference > 0) {

                motorC1.setPower(1);
                motorC2.setPower(-1);
            } else {
 
                motorC1.setPower(-1);
                motorC2.setPower(1);
            }
            

            currentAngle = getYaw();
            
            telemetry.addData("Angulo Atual", "%.2f", currentAngle);
            telemetry.update();
        }

        motorC1.setPower(0);
        motorC2.setPower(0);
        telemetry.addData("Status", "Giro Concluido!");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        
        motorC1 = hardwareMap.get(DcMotor.class, "motorC1");
        motorC2 = hardwareMap.get(DcMotor.class, "motorC2");
        imu = hardwareMap.get(IMU.class, "imu"); 

        motorC2.setDirection(DcMotor.Direction.REVERSE);
        motorC1.setDirection(DcMotor.Direction.FORWARD);


        IMU.Parameters parametros = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );

        imu.initialize(parametros);

        telemetry.addData("Status", "Calibrando e Resetando Yaw...");
        telemetry.update();
        imu.resetYaw(); 

        telemetry.addData("Status", "Pronto. Aguardando a partida.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            
           
            turnToAngle(90.0, 0.3); 
        
            sleep(1000);
            

        }
    }
}
