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

@Autonomous(name="specimen_auto")
public class specimen_auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        CRServo claw = hardwareMap.get(CRServo.class, "claw");
        DcMotorEx pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        waitForStart();

        claw.setPower(-1.0);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(new movePivot(pivot, -1952))
                .waitSeconds(0.5)
                .lineToX(31)
                .waitSeconds(0.5)
                .stopAndAdd(new movePivot(pivot, -1100))
                .waitSeconds(0.3)
                .stopAndAdd(new moveClaw(claw, 1.0))
                .waitSeconds(0.5)
                .lineToX(16)
                .stopAndAdd(new movePivot(pivot, -310))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(16, -30))
                .waitSeconds(0.2)
                .lineToX(53)
                .strafeTo(new Vector2d(53, -37))
                .waitSeconds(0.2)
                .lineToX(11)
                .lineToX(50)
                .strafeTo(new Vector2d(50, -47))
                .waitSeconds(0.2)
                .lineToX(11)
//                .waitSeconds(0.2)
                .lineToX(53)
                .strafeTo(new Vector2d(50, -56))
                .waitSeconds(0.2)
                .lineToX(4)
                .lineToX(10)
                .turnTo(180)
                .build()
        );
    }

    public class moveClaw implements Action {

        CRServo claw;
        double power;

        public moveClaw(CRServo s, double power) {
            this.claw = s;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPower(power);
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
