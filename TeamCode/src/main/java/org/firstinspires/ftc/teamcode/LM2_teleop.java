package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LM2 teleop)", group="Robot")

public class LM2_teleop extends LinearOpMode {

    public DcMotor left_front = null;
    public DcMotor right_front = null;
    public DcMotor left_back = null;
    public DcMotor right_back = null;

    public double speed = 1.0;

    @Override
    public void runOpMode() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            left_front.setPower(gamepad1.left_stick_y);
            left_back.setPower(-gamepad1.left_stick_y);
            right_front.setPower(gamepad1.right_stick_y);
            right_back.setPower(-gamepad1.right_stick_y);

            // turn left (speed = 1.0)
            if(gamepad1.left_bumper){
                left_front.setPower(-speed);
                left_back.setPower(-speed);
                right_front.setPower(speed);
                right_back.setPower(speed);
            }

            // turn right (speed = 1.0)
            if(gamepad1.right_bumper){
                left_front.setPower(speed);
                left_back.setPower(speed);
                right_front.setPower(-speed);
                right_back.setPower(-speed);
            }

            // strafe right (speed = 1.0)
            if(gamepad1.right_stick_x > 0){
                left_front.setPower(speed);
                left_back.setPower(-speed);
                right_front.setPower(-speed);
                right_back.setPower(speed);
            }

            // strafe left (speed = 1.0)
            if(gamepad1.right_stick_x > 0){
                left_front.setPower(-speed);
                left_back.setPower(speed);
                right_front.setPower(speed);
                right_back.setPower(-speed);
            }

        }
    }
}
