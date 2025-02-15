package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="current_teleop", group="Robot")

public class current_teleop extends LinearOpMode {

    public DcMotor left_front = null;
    public DcMotor right_front = null;
    public DcMotor left_back = null;
    public DcMotor right_back = null;
    private Servo claw = null;
    private DcMotor extension = null;
    private DcMotorEx pivot = null;

    private double INTAKE_IN_POWER = 1.0;
    private double INTAKE_OUT_POWER = -1.0;
    private double INTAKE_OFF_POWER = 0.0;

    private double intakePower = INTAKE_OFF_POWER;


    private double EXTENSION_OUT_POWER = 1.0;
    private double EXTENSION_IN_POWER = -1.0;
    private double EXTENSION_OFF_POWER = 0.0;

    private double extensionPower = EXTENSION_OFF_POWER;

    private int pivot_target_pos;
    private int pivot_home_pos;

    private double PIVOT_UP_POWER = 0.25;
    private double PIVOT_DOWN_POWER = -0.0125;
    private double PIVOT_HOLD_POWER = 0.001;
    public enum PivotModes {UP, HOLD, DOWN};
    public PivotModes pivotMode;


    @Override
    public void runOpMode() {
        left_front = hardwareMap.get(DcMotor.class, "leftFront");
        right_front = hardwareMap.get(DcMotor.class, "rightFront");
        left_back = hardwareMap.get(DcMotor.class, "leftBack");
        right_back = hardwareMap.get(DcMotor.class, "rightBack");
        claw = hardwareMap.get(Servo.class, "claw");
        extension = hardwareMap.get(DcMotor.class, "extension");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        claw.setDirection(Servo.Direction.FORWARD); // Forward should INTAKE.
        extension.setDirection(DcMotor.Direction.REVERSE); // Forward should EXTEND.
        pivot.setDirection(DcMotor.Direction.REVERSE); // Forward should pivot UP, or away from the stowed position.

//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot_home_pos = 0;

        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        while (opModeIsActive()) {

            double speed = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double  strafe = gamepad1.left_stick_x;

            double leftFrontMovement = speed + turn + strafe;
            double rightFrontMovement = speed - turn - strafe;
            double leftBackMovement = speed + turn - strafe;
            double rightBackMovement = speed - turn + strafe;

            left_front.setPower(leftFrontMovement);
            right_front.setPower(rightFrontMovement);
            left_back.setPower(leftBackMovement);
            right_back.setPower(rightBackMovement);

            boolean intakeInButton = gamepad2.a;
            boolean intakeOutButton = gamepad2.y;
            boolean intakeOffButton = gamepad2.x;

            // This conditional reduces ambiguity when multiple buttons are pressed.
            if (intakeInButton && intakeOutButton) {
                intakeInButton = false;
            } else if (intakeOffButton && (intakeInButton || intakeOutButton)) {
                intakeInButton = intakeOutButton = false;
            }

            boolean extensionOutButton = gamepad2.left_trigger > 0.2;
            boolean extensionInButton = gamepad2.left_bumper;
            if (extensionOutButton && extensionInButton) {
                extensionOutButton = false;
            }

            boolean pivotUpButton = gamepad2.right_bumper;
            boolean pivotDownButton = gamepad2.right_trigger > 0.2;
            if (pivotUpButton && pivotDownButton) {
                pivotUpButton = false;
            }

            // INTAKE CODE
            if (gamepad2.a) {
                claw.setPosition(0.7);
            }
            else if (gamepad2.b) {
                claw.setPosition(-0.5);
            }

            // EXTENSION CODE
            double extensionPower;
            if (extensionOutButton) {
                extensionPower = EXTENSION_OUT_POWER;
            } else if (extensionInButton) {
                extensionPower = EXTENSION_IN_POWER;
            } else {
                extensionPower = 0;
            }

            // Determine pivot mode
            if (pivotUpButton) {
                pivotMode = PivotModes.UP;
                pivot_target_pos += 5;
            } else if (pivotDownButton) {
                pivotMode = PivotModes.DOWN;
                pivot_target_pos -= 5;
            } else {
                pivotMode = PivotModes.HOLD;
            }

            double pivotPower;
        if (pivotMode == PivotModes.UP) {
           pivotPower = PIVOT_UP_POWER;
        } else if (pivotMode == PivotModes.DOWN) {
            pivotPower = PIVOT_DOWN_POWER;
        } else {
            pivotPower = PIVOT_HOLD_POWER;
        }

//            intake.setPower(intakePower);
//            extension.setPower(extensionPower);
//            pivot.setTargetPosition(pivot_target_pos);
//            pivot.setPower(1.0);

            String pivot_mode_str;
            if (pivotMode == PivotModes.UP) {
                pivot_mode_str = "UP";
            } else if (pivotMode == PivotModes.DOWN) {
                pivot_mode_str = "DOWN";
            } else {
                pivot_mode_str = "HOLD";
            }
            // UPDATE TELEMETRY
            telemetry.addData("Intake", "%%4.2f", intakePower);
            telemetry.addData("Extension", "%4.2f", extension.getPower());
            telemetry.addData("Pivot Current/Target/power", "%d, %d, %4.2f", pivot.getCurrentPosition(), pivot.getTargetPosition(),pivot.getPower());
            telemetry.addData("Pivot MODE", "%s", pivot_mode_str);
            telemetry.update();
        }

    }
}
