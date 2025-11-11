package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp Mecanum Completo")
public class TeleOP extends OpMode {

    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotor Intake, Shooter;

    private int robotState = 0;

    private boolean previousLBValue = false;
    private boolean previousRBValue = false;
    private boolean previousXbuttonValue = false;
    private boolean previousYbuttonValue = false;
    private boolean previousAButtonValue = false;
    private boolean previousBButtonValue = false;

    private long lastChangeTime = 0;
    private final long debounceDelay = 300;

    @Override
    public void init() {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        // Direções dos motores
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("TeleOp mecanum pronto para iniciar!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ===== MOVIMENTO MECANUM =====
        double y = -gamepad1.left_stick_y;   // frente/trás
        double x = gamepad1.left_stick_x * 1.1; // lateral
        double rx = gamepad1.right_stick_x;  // rotação

        // Fórmula mecanum
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        // Normaliza para não passar de 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                      Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        // ===== AJUSTE DE VELOCIDADE =====
        long now = System.currentTimeMillis();
        if (gamepad1.left_bumper && !previousLBValue && (now - lastChangeTime > debounceDelay)) {
            robotState--;
            lastChangeTime = now;
        }
        if (gamepad1.right_bumper && !previousRBValue && (now - lastChangeTime > debounceDelay)) {
            robotState++;
            lastChangeTime = now;
        }
        if (robotState < 0) robotState = 4;
        if (robotState > 4) robotState = 0;

        double velpot;
        switch (robotState) {
            case 0:
                velpot = 1.0;
                break;
            case 1:
                velpot = 0.75;
                break;
            case 2:
                velpot = 0.5;
                break;
            case 3:
                velpot = 0.25;
                break;
            default:
                velpot = 0.0;
                break;
        }

        // ===== CONTROLE DO INTAKE E SHOOTER =====
        boolean buttonA = gamepad1.a;
        boolean buttonB = gamepad1.b;
        boolean buttonX = gamepad1.x;
        boolean buttonY = gamepad1.y;

        // Intake
        if (buttonA && !previousAButtonValue) {
            Intake.setPower(1.0);
        } else if (buttonB && !previousBButtonValue) {
            Intake.setPower(-1.0);
        } else if (buttonY && !previousYbuttonValue) {
            Intake.setPower(0.0);
        }

        // Shooter
        if (buttonX && !previousXbuttonValue) {
            Shooter.setPower(1.0);
        } else if (buttonY && !previousYbuttonValue) {
            Shooter.setPower(0.0);
        }

        // ===== APLICAÇÃO DE POTÊNCIA =====
        frontLeft.setPower(frontLeftPower * velpot);
        backLeft.setPower(backLeftPower * velpot);
        frontRight.setPower(frontRightPower * velpot);
        backRight.setPower(backRightPower * velpot);

        // ===== ATUALIZA ESTADOS ANTERIORES =====
        previousLBValue = gamepad1.left_bumper;
        previousRBValue = gamepad1.right_bumper;
        previousAButtonValue = buttonA;
        previousBButtonValue = buttonB;
        previousXbuttonValue = buttonX;
        previousYbuttonValue = buttonY;

        // ===== TELEMETRIA =====
        telemetry.addData("Modo de velocidade", robotState);
        telemetry.addData("Velocidade atual", velpot);
        telemetry.addData("Intake", Intake.getPower());
        telemetry.addData("Shooter", Shooter.getPower());
        telemetry.update();
    }
}
