package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Encoder Test ")
public class encoder_test extends OpMode {
    public DcMotor pivot = null;
    int pivotDownPosition = 0;

    int pivotUpPosition = 1900;
    double newTarget;

    @Override
    public void init() {
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        telemetry.addData("Hardware: ", "Initialized");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            pivot.setTargetPosition(pivotUpPosition);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(0.5);
        }
        telemetry.addData("Motor Ticks: ", pivot.getCurrentPosition());
        if (gamepad1.b) {
            pivot.setTargetPosition(pivotDownPosition);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(0.5);
        }
    }
}