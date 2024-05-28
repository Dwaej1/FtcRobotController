package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 *
 */
@TeleOp(name="OmniDrive", group="Linear OpMode")
//@Disabled
public class Comp extends LinearOpMode {

    IMU imu;
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    public DcMotor viperMotor = null;
    public Servo leftClawServo = null;
    public Servo rightClawServo = null;

    /**
     *
     */
    public void runOpMode () {

        imu = hardwareMap.get(IMU.class, "imu");
        YawPitchRollAngles gyro = imu.getRobotYawPitchRollAngles();

        leftFrontMotor = hardwareMap.get(DcMotor.class, "null");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "null");
        leftRearMotor = hardwareMap.get(DcMotor.class, "null");
        rightRearMotor = hardwareMap.get(DcMotor.class, "null");

        viperMotor = hardwareMap.get(DcMotor.class, "viper");
        leftClawServo = hardwareMap.get(Servo.class, "leftClaw");
        rightClawServo = hardwareMap.get(Servo.class, "rightClaw");
        viperMotor.setTargetPosition(0); viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);


        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);

        int yawoffset = 0;
        double maxPower = 1;
        int viperslideopen = 1000;

        double yaw = -(gyro.getYaw(AngleUnit.DEGREES) + yawoffset);

        float leftY = -gamepad1.left_stick_y;
        float leftX = gamepad1.left_stick_x;
        float rx = gamepad1.right_stick_x;

        double[] point = rotatePointByDegrees(leftX, leftY, yaw);
        double x = point[0];
        double y = point[1];

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // button controls
        if (gamepad1.b) {
            //resetIMU
            imu = hardwareMap.get(IMU.class, "imu");
            imu.resetYaw();
        }

        frontLeftPower = accelerate(frontLeftPower, leftFrontMotor, maxPower);
        frontRightPower = accelerate(frontRightPower, rightFrontMotor, maxPower);
        backRightPower = accelerate(backRightPower, rightRearMotor, maxPower);
        backLeftPower = accelerate(backLeftPower, leftRearMotor, maxPower);

        leftFrontMotor.setPower(frontLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightRearMotor.setPower(backRightPower);
        leftRearMotor.setPower(backLeftPower);


        viperMotor = viperslide(viperMotor, viperslideopen, -gamepad2.left_stick_y);
        leftClawServo = clawcontrol(leftClawServo,rightClawServo,true);
        rightClawServo = clawcontrol(leftClawServo,rightClawServo,false);

        //telemetry
        telemetry.addData("Rotated Y: ", "%.2f", y);
        telemetry.addData("Rotated X: ", "%.2f", x);
        telemetry.addData("Yaw: ", "%.2f", yaw);
        telemetry.addData("frontLeftPower: ", "%.2f", frontLeftPower);
        telemetry.addData("backLeftPower: ", "%.2f", backLeftPower);
        telemetry.addData("frontRightPower: ", "%.2f", frontRightPower);
        telemetry.addData("backRightPower: ", "%.2f", backRightPower);
        telemetry.addData("Max Power: ", "%.2f", maxPower);
        telemetry.update();
    }

    /**
     *
     * @param targetPower
     * @param motor
     * @return
     */
    private double accelerate (double targetPower, DcMotor motor, double maxPower) {

        double accelerationIncrement = 0.01;
        double currentPower = motor.getPower();
        double newpower = targetPower;
        if (currentPower < targetPower)
        {
            double power = currentPower + accelerationIncrement;
            if (power > targetPower)
            {
                power = targetPower;
            }

            newpower = power;
        }
        else if (currentPower > targetPower)
        {
            double power = currentPower - accelerationIncrement;
            if (power < targetPower)
            {
                power = targetPower;
            }

            newpower = power;
        }
        if (Math.abs(newpower) > maxPower)
        {
            if (newpower < 0)
            {
                newpower = maxPower * -1;
            }
            else
            {
                newpower = maxPower;
            }
        }
        return newpower;

    }

    public static double[] rotatePointByDegrees (double x, double y, double degrees) {

        double x1 = 0;
        double y1 = 0;

        double x2 = x;
        double y2 = y;

        double radians = degrees * Math.PI / (double)180;

        double x3 = Math.cos(radians) * (x2 - x1) - Math.sin(radians) * (y2 - y1) + x1;
        double y3 = Math.sin(radians) * (x2 - x1) + Math.cos(radians) * (y2 - y1) + y1;

        double[] point = new double[]{roundDecimal(x3,3), roundDecimal(y3, 3)};
        return point;
        //return rotateLineByDegrees(0,0, x, y, degrees);
    }

    public static double roundDecimal(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }

    private static DcMotor viperslide (DcMotor motor, int open, double control) {
        if (control > 0) {
            motor.setTargetPosition(open);
        } else {
            motor.setTargetPosition(25);
        }
        motor.setPower(Math.abs(control));
        return motor;
    }

    private static Servo clawcontrol (Servo leftServo, Servo rightServo, boolean left) {



        if (left) {
            return leftServo;
        } else {
            return rightServo;
        }
    }

}
