package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="myteleop", group="Linear OpMode")
//@Disabled
public class mycode extends LinearOpMode {

    private DcMotor testmotor = null;
    private Servo testservo = null;

    @Override
    public void runOpMode() {

        testmotor = hardwareMap.get(DcMotor.class, "MotorName");
        testmotor.setDirection(DcMotor.Direction.REVERSE);

        testservo = hardwareMap.get(Servo.class, "ServoName");
        testservo.setDirection(Servo.Direction.REVERSE);

        //these are variables and can be named and used for whatever you wish
        double position = 0;
        double speed = 0;

        waitForStart();
        while (opModeIsActive()) {

            if (true) {
                //this will happen
            }
            if (false) {
                //this will be skipped
            }
            if (gamepad1.left_stick_y > .5) {
                position = 0.8;
            }

            testservo.setPosition(position);

            calcspeed(speed);

        }
    }


    public void calcspeed (double power) {

        double leftY = gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;

        power = (leftY * rightX) / 2;

        testmotor.setPower(power);
    }
}