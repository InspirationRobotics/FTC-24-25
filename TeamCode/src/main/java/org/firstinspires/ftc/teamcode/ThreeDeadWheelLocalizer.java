package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double leftYTicks = 1419.2519064440153; // y position of the first parallel encoder (in tick units)
        public double rightYTicks = 912.3171494448577; // y position of the second parallel encoder (in tick units)
        public double centerXTicks = -398.26167100220823; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder left, right, center;

    public final double inPerTick;

    private int lastLeftPos, lastRightPos, lastCenterPos;
    private boolean initialized;
    private Pose2d pose;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d pose) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        left = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront"))); //port 0
        right = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack"))); //port 3
        center = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack"))); //port 1
        // TODO: reverse encoder directions if needed
        left.setDirection(DcMotorEx.Direction.REVERSE);
        right.setDirection(DcMotorEx.Direction.REVERSE);
        center.setDirection(DcMotorEx.Direction.REVERSE);

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);

        this.pose = pose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair par0PosVel = left.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = right.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = center.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastLeftPos = par0PosVel.position;
            lastRightPos = par1PosVel.position;
            lastCenterPos = perpPosVel.position;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        int par0PosDelta = par0PosVel.position - lastLeftPos;
        int par1PosDelta = par1PosVel.position - lastRightPos;
        int perpPosDelta = perpPosVel.position - lastCenterPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.leftYTicks * par1PosDelta - PARAMS.rightYTicks * par0PosDelta) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                                (PARAMS.leftYTicks * par1PosVel.velocity - PARAMS.rightYTicks * par0PosVel.velocity) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.centerXTicks / (PARAMS.leftYTicks - PARAMS.rightYTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.centerXTicks / (PARAMS.leftYTicks - PARAMS.rightYTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                })
        );

        lastLeftPos = par0PosVel.position;
        lastRightPos = par1PosVel.position;
        lastCenterPos = perpPosVel.position;

        pose = pose.plus(twist.value());
        return twist.velocity().value();
    }
}
