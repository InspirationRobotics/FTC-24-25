package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Ruth Teleop", group="Robot")

//@Disabled

public class RuthCode extends LinearOpMode {


    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor  leftFront   = null; //the left drivetrain motor

    public DcMotor  rightFront  = null; //the right drivetrain motor

    public DcMotor  leftBack    = null; //the arm motor

    public DcMotor  rightBack   = null; //the active intake servo







    @Override

    public void runOpMode() {

        /*

        These variables are private to the OpMode, and are used to control the drivetrain.

         */

        double left;

        double right;

        double forward;

        double rotate;







        /* Define and Initialize Motors */

        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); //the left front motor

        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); //the right front motor

        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); //the left back motor

        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); //the right back motor



        /* may need to change*/

        leftFront.setDirection(DcMotor.Direction.FORWARD);

        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setDirection(DcMotor.Direction.FORWARD);

        rightBack.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down

        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot

        stops much quicker. */

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        /* Send telemetry message to signify robot waiting */

        telemetry.addLine("Robot Ready.");

        telemetry.update();



        /* Wait for the game driver to press play */

        waitForStart();



        /* Run until the driver presses stop */

        while (opModeIsActive()) {

            double max;

            /*       1) Axial:    Driving forward and backward               Left-joystick Forward/Backward

                     2) Lateral:  Strafing right and left                     Left-joystick Right and Left

                     3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left*/



            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

            double lateral = gamepad1.left_stick_x;

            double yaw = gamepad1.right_stick_x;



            // Combine the joystick requests for each axis-motion to determine each wheel's power.

            // Set up a variable for each drive wheel to save the power level for telemetry.

            double leftFrontPower = axial + lateral + yaw;

            double rightFrontPower = axial - lateral - yaw;

            double leftBackPower = axial - lateral + yaw;

            double rightBackPower = axial + lateral - yaw;



            // Normalize the values so no wheel power exceeds 100%

            // This ensures that the robot maintains the desired motion.

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));

            max = Math.max(max, Math.abs(leftBackPower));

            max = Math.max(max, Math.abs(rightBackPower));



            if (max > 1.0) {

                leftFrontPower /= max;

                rightFrontPower /= max;

                leftBackPower /= max;

                rightBackPower /= max;

            }



            // This is test code:

            //

            // Uncomment the following code to test your motor directions.

            // Each button should make the corresponding motor run FORWARD.

            //   1) First get all the motors to take to correct positions on the robot

            //      by adjusting your Robot Configuration if necessary.

            //   2) Then make sure they run in the correct direction by modifying the

            //      the setDirection() calls above.

            // Once the correct motors move in the correct direction re-comment this code.



            /*

            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad

            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad

            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad

            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            */



            // Send calculated power to wheels

            leftFront.setPower(leftFrontPower);

            rightFront.setPower(rightFrontPower);

            leftBack.setPower(leftBackPower);

            rightBack.setPower(rightBackPower);



            // Show the elapsed game time and wheel power.

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);

            telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.update();

        }

    }}

