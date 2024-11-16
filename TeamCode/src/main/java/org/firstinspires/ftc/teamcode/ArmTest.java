package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Arm Test")
@Config
public class ArmTest extends OpMode {
    public static Params PARAMS = new Params();

    private FtcDashboard dash = FtcDashboard.getInstance();
    private Grabber grabber;



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        grabber = new Grabber(hardwareMap);
    }

    private long prev_time = System.currentTimeMillis();
    private double slide_position = 0.0;

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // Get execution time
        long dt = System.currentTimeMillis() - prev_time;
        prev_time = System.currentTimeMillis();

        if (gamepad1.right_bumper) {
            grabber.intakeGo();
        } else if (gamepad1.left_bumper) {
            grabber.intakeReverse();
        } else {
            grabber.intakeStop();
        }

        if (gamepad1.x) {
            grabber.armToInside();
        } else if (gamepad1.y) {
            grabber.armToHook();
        } else if (gamepad1.b) {
            grabber.armToFloor();
        }

        slide_position += PARAMS.slide_speed * dt * linearDeadband(gamepad1.left_trigger, 0.1);
        slide_position -= PARAMS.slide_speed * dt * linearDeadband(gamepad1.right_trigger, 0.1);
        slide_position = clamp(slide_position, 0.0, 1.0);
        grabber.slideToPercent(slide_position);

        grabber.run(packet);

        packet.put("dt", dt);
        packet.put("slide_pose", slide_position);
        dash.sendTelemetryPacket(packet);
    }

    private static double linearDeadband(double raw, double deadband) {
        return Math.abs(raw) < deadband ? 0 : Math.signum(raw) * (Math.abs(raw) - deadband) / (1 - deadband);
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static class Params {
        public double slide_speed = 0.0001;
    }
}
