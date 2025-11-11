package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "MECANUM IMU AUTONOMO", group = "FTC")
public class IMU_Mecanum_Autonomous extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    private final double DRIVE_POWER = 0.5;

    @Override
    public void runOpMode() {

        // Inicialização dos motores
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        imu        = hardwareMap.get(BNO055IMU.class, "imu");

        // Direções
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Inicialização do IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.addData("Status", "IMU Pronto. Aguardando o start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Movimento reto por 2 segundos
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 2.0) {
                driveFieldCentric(DRIVE_POWER, 0.0, 0.0);
            }
            stopDrive();

            // Giro de 90 graus
            turnToAngle(90.0, DRIVE_POWER * 0.5);

            stopDrive();
            telemetry.addData("Status", "Autonomo Concluido");
            telemetry.update();
        }
    }

    private void driveFieldCentric(double forward, double right, double rotate) {
        double robotAngle = getHeadingRadians();

        double magnitude = Math.hypot(right, forward);
        double theta = Math.atan2(forward, right);
        theta = theta - robotAngle;

        double newForward = magnitude * Math.sin(theta);
        double newRight = magnitude * Math.cos(theta);

        double fl = newForward + newRight + rotate;
        double fr = newForward - newRight - rotate;
        double bl = newForward - newRight + rotate;
        double br = newForward + newRight - rotate;

        double maxPower = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (maxPower > 1.0) {
            fl /= maxPower;
            fr /= maxPower;
            bl /= maxPower;
            br /= maxPower;
        }

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftBack.setPower(bl);
        rightBack.setPower(br);

        telemetry.addData("Heading", Math.toDegrees(robotAngle));
        telemetry.update();
    }

    private double getHeadingRadians() {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle; // Yaw
    }

    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private void turnToAngle(double targetAngle, double power) {
        double currentAngle, diff;
        double tolerance = 1.0;

        do {
            currentAngle = Math.toDegrees(getHeadingRadians());
            diff = targetAngle - currentAngle;

            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;

            if (Math.abs(diff) > tolerance) {
                double turnPower = Math.copySign(power, diff);
                leftFront.setPower(turnPower);
                leftBack.setPower(turnPower);
                rightFront.setPower(-turnPower);
                rightBack.setPower(-turnPower);
            } else {
                stopDrive();
            }

            telemetry.addData("Atual", currentAngle);
            telemetry.addData("Diff", diff);
            telemetry.update();

        } while (opModeIsActive() && Math.abs(diff) > tolerance);

        stopDrive();
    }
}
