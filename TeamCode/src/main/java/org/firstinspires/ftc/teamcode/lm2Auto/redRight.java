package org.firstinspires.ftc.teamcode.lm2Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class redRight extends LinearOpMode {
    //RIGHT

    public Servo clawLeft = null;
    public Servo clawRight = null;
    public DcMotor wristMotor = null;
    public DcMotor armMotor = null;

    int wristDownPosition = -91;
    // Position of the wrist when it's on the ground (ticks)
    int wristUpPosition = 145;
    // Position of the arm when it's lifted (ticks)
    int armUpPosition = 1900;
    // Position of the arm when it's down (ticks)
    int armDownPosition = 0;

    int armIntPosition = 0;

    int wristIntPosition = 0;



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //put motors and servos here

        // Reset the motor encoder so that it reads zero ticks


        waitForStart();
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(1)
                .forward(2)
                .strafeRight(18)
                .build();
        drive.followTrajectorySequence(trajSeq);
        if (isStopRequested()) return;
    }
}
