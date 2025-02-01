package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "pivot testing")

public class pivotTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Position of the arm when it's lifted (ticks)

        int pivotUpPosition = 1900;

        // Position of the arm when it's down (ticks)

        int pivotDownPosition = 0;

        // Find a motor in the hardware map named "pivot"

        DcMotor pivot = hardwareMap.dcMotor.get("pivot");

        // Reset the motor encoder so that it reads zero ticks

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position

        pivot.setTargetPosition(pivotDownPosition);

        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {

            // If the a button is clicked, it will raise the arm

            if (gamepad1.a) {
                pivot.setTargetPosition(pivotUpPosition);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(0.5);
            }

            // If the b button is clicked, it will lower the arm

            if (gamepad1.b) {
                pivot.setTargetPosition(pivotDownPosition);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(0.5);
            }

            // Get the current position of the pivot

            double position = pivot.getCurrentPosition();

            // Get the target position of the pivot

            double desiredPosition = pivot.getTargetPosition();

            // Show the position of the pivot on telemetry

            telemetry.addData("Encoder Position:", position);

            // Show the target position of the pivot on telemetry

            telemetry.addData("Desired Position:", desiredPosition);
            telemetry.update();
        }
    }
}