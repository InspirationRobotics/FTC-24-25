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
        CRServo claw = hardwareMap.get(CRServo.class, "claw");
        DcMotorEx pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        waitForStart();

//        claw.setPower(-0.4);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new moveClaw(claw, -0.4))
                        .stopAndAdd(new movePivot(pivot, -1952))
                        .lineToX(31)
                        .stopAndAdd(new movePivot(pivot, -1200))
                        .waitSeconds(0.1)
                        .stopAndAdd(new moveClaw(claw, 1.0))
                        .lineToX(16)
                        .stopAndAdd(new movePivot(pivot, -500))
                        .strafeToConstantHeading(new Vector2d(16, -30))
                        .waitSeconds(0.01)
                        .lineToX(49)
                        .strafeTo(new Vector2d(49, -39))
                        .waitSeconds(0.01)
                        .lineToX(15)
                        .lineToXLinearHeading(20, 135)
                        .stopAndAdd(new movePivot(pivot, -680))
                        .lineToX(0)
                        .stopAndAdd(new moveClaw(claw, -0.4))
                        .waitSeconds(0.3)
                        .stopAndAdd(new movePivot(pivot, -1952))
                        .lineToXLinearHeading(20, 0)
                        .strafeToConstantHeading(new Vector2d(20, 9))
                        .waitSeconds(0.01)
                        .lineToX(32)
                        .stopAndAdd(new movePivot(pivot, -1100))
                        .waitSeconds(0.2)
                        .stopAndAdd(new moveClaw(claw, 1.0))
                        .waitSeconds(0.4)
                        .lineToX(14)
                        .turnTo(-135)
                        .waitSeconds(0.1)
                        .strafeTo(new Vector2d(14, -39))
                        .waitSeconds(0.01)
                        .stopAndAdd(new movePivot(pivot, -660))
                        .waitSeconds(0.2)
                        .lineToX(8)
                        .stopAndAdd(new moveClaw(claw, -0.4))
                        .waitSeconds(0.3)
                        .stopAndAdd(new movePivot(pivot, -1952))
                        .lineToXLinearHeading(20, -50)
                        .stopAndAdd(new movePivot(pivot, -1100))
                        .strafeToConstantHeading(new Vector2d(20, 15))
                        .waitSeconds(0.01)
                        .stopAndAdd(new moveClaw(claw, 1.0))
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

