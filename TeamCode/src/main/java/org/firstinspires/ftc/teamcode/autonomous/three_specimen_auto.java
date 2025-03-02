package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="three_specimen_auto")
public class three_specimen_auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotorEx pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        waitForStart();

        claw.setPosition(-1.0);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new movePivot(pivot, -1952))
                        .waitSeconds(0.5)
                        .lineToX(31)
                        .stopAndAdd(new movePivot(pivot, -1100))
                        .waitSeconds(0.4)
                        .stopAndAdd(new moveClaw(claw, 0.7))
                        .waitSeconds(0.5)
                        .lineToX(16)
                        .stopAndAdd(new movePivot(pivot, -310))
                        .waitSeconds(0.5)
                        .strafeToConstantHeading(new Vector2d(16, -30))
                        .waitSeconds(0.2)
                        .lineToX(53)
                        .strafeTo(new Vector2d(47, -37))
                        .waitSeconds(0.2)
                        .lineToX(11)
                        .lineToX(50)
                        .strafeTo(new Vector2d(47, -47))
                        .waitSeconds(0.2)
                        .lineToX(8)
                        .lineToX(12)
                        .turnTo(135)
                        .stopAndAdd(new movePivot(pivot, -460))
                        .lineToX(-2)
                        .stopAndAdd(new moveClaw(claw, -1.0))
                        .waitSeconds(0.2)
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

