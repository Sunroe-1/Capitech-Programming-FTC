package org.firstinspires.ftc.teamcode;
//Alteracoes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Calibrar", group = "Calibration")
public class Calibrar extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Constantes
    static final double POWER = 0.8;           // potência de teste
    static final int TEMPO_MS = 2000;          // duração do teste em milissegundos

    @Override
    public void runOpMode() {

        // Mapeamento dos motores
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Configurações iniciais
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("🔧 Pronto para teste dos motores.");
        telemetry.addLine("Pressione PLAY para iniciar.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Teste motor FRONT LEFT
            testMotor(frontLeft, "Front Left");

            // Teste motor FRONT RIGHT
            testMotor(frontRight, "Front Right");

            // Teste motor BACK LEFT
            testMotor(backLeft, "Back Left");

            // Teste motor BACK RIGHT
            testMotor(backRight, "Back Right");

            telemetry.addLine("✅ Teste completo!");
            telemetry.addLine("Compare os valores de ticks no log acima.");
            telemetry.update();

            sleep(5000);
        }
    }

    private void testMotor(DcMotor motor, String nome) {
        telemetry.addLine("🌀 Testando motor: " + nome);
        telemetry.update();

        // Resetar encoder
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ligar motor
        motor.setPower(POWER);
        sleep(TEMPO_MS);
        motor.setPower(0);

        // Ler posição final
        int posicao = motor.getCurrentPosition();

        telemetry.addData("Motor", nome);
        telemetry.addData("Ticks após %d ms", TEMPO_MS);
        telemetry.addData("Valor", posicao);
        telemetry.addLine("-------------------------------");
        telemetry.update();

        sleep(1000);
    }
}
