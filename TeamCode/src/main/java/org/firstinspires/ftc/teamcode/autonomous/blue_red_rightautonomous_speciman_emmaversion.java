/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

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

@Autonomous(name="blue/red_RIGHTautonomous_speciman (emma's version)", group="Robot")

public class blue_red_rightautonomous_speciman_emmaversion extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor left_front = null;
    public DcMotor right_front = null;
    public DcMotor left_back = null;
    public DcMotor right_back = null;
    public DcMotor extension = null;
    private DcMotorEx pivot = null;
    private CRServo intake = null;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

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

        // Initialize the drive system variables.
        left_front = hardwareMap.get(DcMotor.class, "leftFront");
        right_front = hardwareMap.get(DcMotor.class, "rightFront");
        left_back = hardwareMap.get(DcMotor.class, "leftBack");
        right_back = hardwareMap.get(DcMotor.class, "rightBack");

        intake = hardwareMap.get(CRServo.class, "intake");
        extension = hardwareMap.get(DcMotor.class, "extension");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        intake.setDirection(CRServo.Direction.FORWARD);
        extension.setDirection(DcMotor.Direction.FORWARD);
        pivot.setDirection(DcMotor.Direction.FORWARD);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.FORWARD);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();


        //Step 1: pivot motor goes up for 1.75 seconds
        pivot.setPower(PIVOT_DOWN_POWER);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.75)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    pivot.setPower(0);

        //Step 2: robot goes forward for 1.45 seconds to be parallel with the bars
        left_front.setPower(FORWARD_SPEED);
        right_front.setPower(FORWARD_SPEED);
        left_back.setPower(FORWARD_SPEED);
        right_back.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.45)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);


        //Step 3: pivot moves down
        pivot.setPower(PIVOT_UP_POWER);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.25)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        pivot.setPower(0);

        sleep(1500);

        //Step 4: robot goes backwards for .75 seconds
         left_front.setPower(-FORWARD_SPEED);
        right_front.setPower(-FORWARD_SPEED);
        left_back.setPower(-FORWARD_SPEED);
        right_back.setPower(-FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .75)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);

        //Step 5: strafe right 2.5 sec to park
        left_front.setPower(FORWARD_SPEED);
        right_front.setPower(-FORWARD_SPEED);
        left_back.setPower(-FORWARD_SPEED);
        right_back.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
