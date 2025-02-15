package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="specimen_auto")
public class specimen_auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo claw = hardwareMap.servo.get("claw");
        DcMotorEx pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        waitForStart();

        claw.setPosition(-0.5);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(new movePivot(pivot, -310))
                .waitSeconds(0.5)
                .lineToX(18)
                .stopAndAdd(new movePivot(pivot, -1952))
                .waitSeconds(1)
                .lineToX(31)
                .waitSeconds(1)
                .stopAndAdd(new movePivot(pivot, -1500))
                .waitSeconds(0.5)
                .stopAndAdd(new moveClaw(claw, 0.7))
                .waitSeconds(0.5)
                .lineToX(16)
                .stopAndAdd(new movePivot(pivot, -310))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(16, -40))
                .waitSeconds(0.5)
                .build()
        );
    }

    public class moveClaw implements Action {

        Servo claw;
        double position;

        public moveClaw(Servo s, double pos) {
            this.claw = s;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(position);
            return false;
        }
    }

    public class movePivot implements Action {

        DcMotorEx pivot;
        int position;

        public movePivot(DcMotorEx motor, int pos) {
            this.pivot = motor;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pivot.setTargetPosition(position);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(1.0);
            return false;
        }
    }
}
