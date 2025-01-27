package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Encoder Test ")
public class encoder_test extends OpMode {
    public DcMotor pivot = null;
    double ticks = 1425.1;
    double newTarget;
    @Override
    public void init() {
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        telemetry.addData("Hardware: ", "Initialized");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            encoder(-1);
        }
        telemetry.addData("Motor Ticks: ", pivot.getCurrentPosition());
        if(gamepad1.b){
            tracker();
        }

    }
    public void encoder(int turnage){
        newTarget = ticks/turnage;
        pivot.setTargetPosition((int)newTarget);
        pivot.setPower(0.8);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void tracker(){
        pivot.setTargetPosition(0);
        pivot.setPower(0.8);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}