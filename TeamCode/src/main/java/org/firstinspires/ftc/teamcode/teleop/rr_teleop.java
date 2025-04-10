package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="rr_teleop", group="Robot")

public class rr_teleop extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    public DcMotor left_front = null;
    public DcMotor right_front = null;
    public DcMotor left_back = null;
    public DcMotor right_back = null;
    private Servo claw = null;
    public DcMotor extension = null;
    private DcMotorEx pivot = null;

    private int pivot_target_pos;
    public int pivot_home_pos;

    private double EXTENSION_OUT_POWER = 1.0;
    private double EXTENSION_IN_POWER = -1.0;

    private double EXTENSION_OFF_POWER = 0.0;
    double extension_power = EXTENSION_OFF_POWER;

    double pivotPower;
    private double PIVOT_UP_POWER = 0.8;
    private double PIVOT_DOWN_POWER = -0.7;
    private double PIVOT_HOLD_POWER = 0.010;

    private enum PivotModes {UP, HOLD, DOWN};
    private rr_teleop.PivotModes pivotMode;
    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "leftFront");
        right_front = hardwareMap.get(DcMotor.class, "rightFront");
        left_back = hardwareMap.get(DcMotor.class, "leftBack");
        right_back = hardwareMap.get(DcMotor.class, "rightBack");
        claw = hardwareMap.get(Servo.class, "claw");
        extension = hardwareMap.get(DcMotor.class, "extension");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");


        //right motors reversed for movement
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot_home_pos = 0;
        extension.setPower(extension_power);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // controls for driver 1
        boolean forwardButton = gamepad1.left_stick_y < -0.1;
        boolean backwardButton = gamepad1.left_stick_y > 0.1;
        boolean strafeRightButton = gamepad1.left_stick_x > 0.1;
        boolean strafeLeftButton = gamepad1.left_stick_x < -0.1;
        boolean turnRightButton = gamepad1.right_stick_x > 0.1;
        boolean turnLeftButton = gamepad1.right_stick_x < -0.1;

        // move forward
        if (forwardButton) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(1.0)),
                    new InstantAction(() -> right_front.setPower(1.0)),
                    new InstantAction(() -> left_back.setPower(1.0)),
                    new InstantAction(() -> right_back.setPower(1.0))
            ));
        }
        else if (gamepad1.left_stick_y == 0)
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(0.0)),
                    new InstantAction(() -> right_front.setPower(0.0)),
                    new InstantAction(() -> left_back.setPower(0.0)),
                    new InstantAction(() -> right_back.setPower(0.0))
            ));

        // move backwards
        if (backwardButton) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(-1.0)),
                    new InstantAction(() -> right_front.setPower(-1.0)),
                    new InstantAction(() -> left_back.setPower(-1.0)),
                    new InstantAction(() -> right_back.setPower(-1.0))
            ));
        }
        else if (gamepad1.left_stick_y == 0)
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(0.0)),
                    new InstantAction(() -> right_front.setPower(0.0)),
                    new InstantAction(() -> left_back.setPower(0.0)),
                    new InstantAction(() -> right_back.setPower(0.0))
            ));

        // strafe right
        if (strafeRightButton) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(1.0)),
                    new InstantAction(() -> right_front.setPower(-1.0)),
                    new InstantAction(() -> left_back.setPower(-1.0)),
                    new InstantAction(() -> right_back.setPower(1.0))
            ));
        }
        else if (gamepad1.left_stick_x == 0)
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(0.0)),
                    new InstantAction(() -> right_front.setPower(0.0)),
                    new InstantAction(() -> left_back.setPower(0.0)),
                    new InstantAction(() -> right_back.setPower(0.0))
            ));
        // strafe left
        if (strafeLeftButton) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(-1.0)),
                    new InstantAction(() -> right_front.setPower(1.0)),
                    new InstantAction(() -> left_back.setPower(1.0)),
                    new InstantAction(() -> right_back.setPower(-1.0))
            ));
        }
        else if (gamepad1.left_stick_x == 0)
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(0.0)),
                    new InstantAction(() -> right_front.setPower(0.0)),
                    new InstantAction(() -> left_back.setPower(0.0)),
                    new InstantAction(() -> right_back.setPower(0.0))
            ));
        // turn right
        if (turnRightButton) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(1.0)),
                    new InstantAction(() -> right_front.setPower(-1.0)),
                    new InstantAction(() -> left_back.setPower(1.0)),
                    new InstantAction(() -> right_back.setPower(-1.0))
            ));
        }
        else if (gamepad1.right_stick_x == 0)
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(0.0)),
                    new InstantAction(() -> right_front.setPower(0.0)),
                    new InstantAction(() -> left_back.setPower(0.0)),
                    new InstantAction(() -> right_back.setPower(0.0))
            ));
        // turn left
        if (turnLeftButton) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(-1.0)),
                    new InstantAction(() -> right_front.setPower(1.0)),
                    new InstantAction(() -> left_back.setPower(-1.0)),
                    new InstantAction(() -> right_back.setPower(1.0))
            ));
        }
        else if (gamepad1.right_stick_x == 0)
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> left_front.setPower(0.0)),
                    new InstantAction(() -> right_front.setPower(0.0)),
                    new InstantAction(() -> left_back.setPower(0.0)),
                    new InstantAction(() -> right_back.setPower(0.0))
            ));

        // controls for driver 2
        boolean clawOpenButton = gamepad2.a;
        boolean clawCloseButton = gamepad2.b;

        boolean extensionOutButton = gamepad2.left_trigger > 0.2;
        boolean extensionInButton = gamepad2.left_bumper;

        boolean pivotUpButton = gamepad2.right_bumper;
        boolean pivotDownButton = gamepad2.right_trigger > 0.2;

        if (clawOpenButton) {
            claw.setPosition(0.7);
        }
        else if (clawCloseButton) {
            claw.setPosition(-0.5);
        }

        // EXTENSION CODE
        if (extensionOutButton) {
            extension_power = EXTENSION_OUT_POWER;
        } else if (extensionInButton) {
            extension_power = EXTENSION_IN_POWER;
        } else {
            extension_power = EXTENSION_OFF_POWER;
        }

        double extensionPower;
        if (extensionOutButton) {
            extensionPower = EXTENSION_OUT_POWER;
        } else if (extensionInButton) {
            extensionPower = EXTENSION_IN_POWER;
        } else {
            extensionPower = 0;
        }

        // PIVOT CODE
        if (pivotUpButton) {
            pivotMode = rr_teleop.PivotModes.UP;
            pivot_target_pos += 5;
        } else if (pivotDownButton) {
            pivotMode = rr_teleop.PivotModes.DOWN;
            pivot_target_pos -= 5;
        } else {
            pivotMode = rr_teleop.PivotModes.HOLD;
        }

        if (pivotMode == rr_teleop.PivotModes.UP) {
            pivotPower = PIVOT_UP_POWER;
        } else if (pivotMode == rr_teleop.PivotModes.DOWN) {
            pivotPower = PIVOT_DOWN_POWER;
        } else {
            pivotPower = PIVOT_HOLD_POWER;
        }

        extension.setPower(extensionPower);
        pivot.setTargetPosition(pivot_target_pos);
        pivot.setPower(pivotPower);




        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}