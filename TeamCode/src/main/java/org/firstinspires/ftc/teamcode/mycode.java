package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="OmniDrive", group="Linear OpMode")
//@Disabled
public class mycode extends LinearOpMode {

    private DcMotor testmotor = null;

    @Override
    public void runOpMode() {

        testmotor = hardwareMap.get(DcMotor.class, "Name");
        testmotor.setDirection(DcMotor.Direction.FORWARD);

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
                double speed = 0.8;
            }

            testmotor.setPower(speed);

        }
    }
}