### `IMU_Mecanum_Autonomous.java`

```java
package org.firstinspires.ftc.teamcode;

// Importações necessárias para Autônomo (LinearOpMode) e Hardware
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.ElapsedTime; // Para controle baseado em tempo, se necessário

/**
 * OpMode Autônomo Mecanum Centrado no Campo com IMU.
 * Esta OpMode utiliza o IMU para navegação baseada em ângulos absolutos (Field Centric).
 */
@Autonomous(name = "MECANUM IMU AUTONOMO", group = "FTC")
public class IMU_Mecanum_Autonomous extends LinearOpMode {

    // 1. Variáveis de Hardware
    private DcMotor leftFront; // leftFrontMotor
    private DcMotor rightFront; // rightFrontMotor
    private DcMotor leftBack; // leftBackMotor
    private DcMotor rightBack; // rightBackMotor
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    // Variáveis de calibração (ajustar conforme a configuração do seu robô)
    private final double DRIVE_POWER = 0.5;

    // --- Métodos de Ajuda ---

    /**
     * Retorna o ângulo Yaw atual do robô em RADIANOS.
     */
    private double getRobotAngleRadians() {
        // Obtém o Yaw, que é o ângulo de rotação em torno do eixo Z
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    
    /**
     * Aplica a lógica Field Centric para calcular as potências dos motores.
     * Deve ser chamada repetidamente em um loop while (opModeIsActive()).
     * 
     * @param fieldForward Potência de movimento para frente/trás (relativa ao campo).
     * @param fieldRight Potência de strafe para direita/esquerda (relativa ao campo).
     * @param rotate Potência de rotação (relativa ao robô).
     */
    private void applyFieldCentricDrive(double fieldForward, double fieldRight, double rotate) {
        
        // Obter o ângulo atual do robô (Yaw)
        double robotAngle = getRobotAngleRadians();
        
        // 1. Conversão para Coordenadas Polares (magnitude e ângulo)
        // Magnitude (r) é a hipotenusa, a força total de movimento
        double magnitude = Math.hypot(fieldRight, fieldForward);
        // Theta é o ângulo desejado no referencial do campo
        double theta = Math.atan2(fieldForward, fieldRight);

        // 2. Rotação do Ângulo: Ajustar o ângulo de movimento relativo ao campo 
        // pela orientação atual do robô (Field Centric Logic)
        theta = AngleUnit.normalizeRadians(theta - robotAngle);
        
        // 3. Conversão de volta para Cartesianas (agora relativas ao robô)
        // Estas são as novas entradas X (newRight) e Y (newForward) no referencial do robô
        double newForward = magnitude * Math.sin(theta);
        double newRight = magnitude * Math.cos(theta);
        
        // 4. Cálculo das Potências dos Motores (Mecanum Robot-Centric)
        // Utilizamos as novas coordenadas (newForward, newRight) + Rotação
        double frontLeftPower  = newForward + newRight + rotate;
        double frontRightPower = newForward - newRight - rotate;
        double backLeftPower   = newForward - newRight + rotate;
        double backRightPower  = newForward + newRight - rotate;

        // Normalização opcional para garantir que a potência não exceda 1.0
        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // 5. Aplicar Potência aos Motores
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
        
        telemetry.addData("Heading", AngleUnit.DEGREES.fromRadians(robotAngle));
        telemetry.update();
    }

    /**
     * Método Sequencial principal (LinearOpMode)
     */
    @Override
    public void runOpMode() {
        
        // --- INICIALIZAÇÃO DE HARDWARE ---
        
        // Mapeamento dos motores (Nomes devem corresponder à Configuração!)
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");
        imu         = hardwareMap.get(IMU.class, "imu");

        // Configuração dos motores (Definir direções e modo de encoder)
        // É comum inverter o lado direito/esquerdo para movimento Mecanum padrão
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Usamos RUN_WITHOUT_ENCODER para garantir que o poder do PID não seja limitado 
        // (embora RUN_USING_ENCODER seja comum, RUN_WITHOUT_ENCODER é sugerido quando a potência é controlada por um loop externo, como aqui ou em PID)
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Configuração da Orientação do IMU
        RevHubOrientationOnRobot RevOrientation = 
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, 
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        
        imu.initialize(new IMU.Parameters(RevOrientation));

        telemetry.addData("Status", "Calibrando e Resetando Yaw...");
        telemetry.update();
        
        // Zera o ângulo de Yaw, definindo a orientação atual como 0 graus (frente do campo)
        imu.resetYaw();
        
        telemetry.addData("Status", "IMU Pronto. Aguardando a partida.");
        telemetry.update();

        // Espera pelo início da partida
        waitForStart();
        
        // O código autônomo executa aqui, sequencialmente
        if (opModeIsActive()) {
            
            // --- MOVIMENTO 1: Dirigir Reto no Campo (Field Centric) por 2 segundos ---
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                // Queremos mover 100% PARA FRENTE (Field Forward = 1.0)
                // Não queremos strafe (Field Right = 0.0)
                // Não queremos rotação (Rotate = 0.0)
                applyFieldCentricDrive(DRIVE_POWER, 0.0, 0.0);
            }
            
            // Parar motores após o movimento
            stopDrive();

            // --- MOVIMENTO 2: Girar 90 graus (IMU based Turn) ---
            // Adaptado da lógica de giro presente em IMUteste
            turnToAngle(90.0, DRIVE_POWER * 0.5); 
            sleep(500); // Pausa

            // --- FIM DO AUTÔNOMO ---
            stopDrive();
            telemetry.addData("Status", "Autonomo Concluido");
            telemetry.update();
        }
    }
    
    /**
     * Método para Parar os Motores
     */
    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    
    /**
     * Roda o robô para um ângulo alvo usando controle de potência simples (Mecanum Turn)
     * Baseado na lógica de giro do IMUteste.
     * Note: Este é um giro simples, não um PID robusto.
     */
    public void turnToAngle(double targetAngle, double power) {

        double currentAngle;
        double angleDifference;
        double tolerance = 1.0; // Tolerância de 1 grau

        // Loop principal: executa enquanto o robô estiver ativo e o erro for maior que a tolerância
        do {
            currentAngle = getRobotAngleRadians(); 
            angleDifference = AngleUnit.DEGREES.fromRadians(targetAngle - AngleUnit.DEGREES.toRadians(AngleUnit.DEGREES.fromRadians(currentAngle))); 
            
            // Normalizar a diferença de ângulo para o intervalo [-180, 180]
            while (angleDifference > 180) angleDifference -= 360;
            while (angleDifference < -180) angleDifference += 360;

            if (Math.abs(angleDifference) > tolerance) {
                // Determina a direção e aplica a potência de giro
                double turnPower = Math.copySign(power, angleDifference);
                
                // Aplica potência diferencial para girar (Mecanum: motores diagonais em direções opostas)
                leftFront.setPower(turnPower); 
                rightFront.setPower(-turnPower); 
                leftBack.setPower(turnPower); 
                rightBack.setPower(-turnPower); 
            } else {
                // Parar quando a tolerância for atingida
                stopDrive();
            }

            // Reporta o status
            telemetry.addData("Angulo Atual", "%.2f", AngleUnit.DEGREES.fromRadians(currentAngle));
            telemetry.addData("Diferença", "%.2f", angleDifference);
            telemetry.update();

        } while (opModeIsActive() && Math.abs(angleDifference) > tolerance);

        stopDrive();
        telemetry.addData("Status", "Giro Concluido!");
        telemetry.update();
    }
}