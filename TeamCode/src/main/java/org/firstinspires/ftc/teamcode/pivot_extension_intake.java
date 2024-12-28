package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="pivot_extension_intake (emma's version)", group="Robot")

public class pivot_extension_intake extends LinearOpMode {


    public DcMotor extension = null;
    private DcMotorEx pivot = null;
    private CRServo intake = null;

    private ElapsedTime     runtime = new ElapsedTime();

    private double INTAKE_IN_POWER = 1.0;
    private double INTAKE_OUT_POWER = -1.0;
    private double INTAKE_OFF_POWER = 0.0;

    private double EXTENSION_OUT_POWER = 1.0;
    private double EXTENSION_IN_POWER = -1.0;

    private double EXTENSION_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;
    double extensionPower = EXTENSION_OFF_POWER;

    double pivotPower;
    private double PIVOT_UP_POWER = 0.8;
    private double PIVOT_DOWN_POWER = -0.7;
    private double PIVOT_HOLD_POWER = 0.010;

    private enum PivotModes {UP, HOLD, DOWN};



    @Override
    public void runOpMode() {


        intake = hardwareMap.get(CRServo.class, "intake");
        extension = hardwareMap.get(DcMotor.class, "extension");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");



        intake.setDirection(CRServo.Direction.FORWARD);
        extension.setDirection(DcMotor.Direction.FORWARD);
        pivot.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        waitForStart();

        pivot.setPower(PIVOT_DOWN_POWER);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        extension.setPower(EXTENSION_OUT_POWER);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        intake.setPower(INTAKE_IN_POWER);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        pivot.setPower(0);
        extension.setPower(0);
        intake.setPower(0);
    }


}