package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "8081 TeleOp - 2024")
@Config
public class teleop8081 extends OpMode {
    public static Params PARAMS = new Params();

    private FtcDashboard dash = FtcDashboard.getInstance();

    private MecanumDrive mecanumDrive;
    private Elevator elevator;
    private Grabber grabber;

    private long prev_time = System.currentTimeMillis();
    private double slide_position = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        elevator = new Elevator(hardwareMap);
        grabber = new Grabber(hardwareMap);
    }

    private boolean ranOnce = false;

    @Override
    public void loop() {
        if (!ranOnce) {
            elevator.setHookOutside();
            ranOnce = true;
        }


        TelemetryPacket packet = new TelemetryPacket();

        // Update pose estimator
        mecanumDrive.updatePoseEstimate();
        double maxDriveSpeed = 1.0 - map(clamp(elevator.getCurrentHeight() / 4000.0, 0.0, 1.0), 0.0, 1.0, 0.0, 0.7);
        double maxRotateSpeed = 0.75 - map(clamp(elevator.getCurrentHeight() / 4000.0, 0.0, 1.0), 0.0, 1.0, 0.0, 0.4);
        mecanumDrive.driveWithController(gamepad1, maxDriveSpeed, maxRotateSpeed);

        // Get execution time
        long dt = System.currentTimeMillis() - prev_time;
        prev_time = System.currentTimeMillis();

        grabber.intakeSetPower(linearDeadband(gamepad1.right_trigger, 0.1) + (gamepad1.right_bumper ? 1.0 : 0.0) * 0.5  - linearDeadband(gamepad1.left_trigger, 0.1));

        if (gamepad1.a) {
            grabber.armToFloor();
        } else if (gamepad1.y) {
            grabber.armToHook();
        } else if (gamepad1.b) {
            grabber.armToPrepareToHook();
        } else if (gamepad1.x) {
            grabber.armToInside();
        }

        slide_position += PARAMS.slide_speed * dt * (gamepad1.right_bumper ? 1.0 : 0.0);
        slide_position -= PARAMS.slide_speed * dt * (gamepad1.left_bumper ? 1.0 : 0.0);
        slide_position = clamp(slide_position, 0.0, 1.0);
        grabber.slideToPercent(slide_position);


        if (gamepad1.dpad_down) {
            elevator.setHeight(0);
        } else if (gamepad1.dpad_left) {
            elevator.setHeight(1400);
        } else if (gamepad1.dpad_right) {
            elevator.setHeight(2000);
        } else if (gamepad1.dpad_up) {
            elevator.setHeight(4000);
        } else if (gamepad2.dpad_up) {
            elevator.setHeight(2000);
            elevator.setHookOutside();
        } else if (gamepad2.dpad_right) {
            elevator.setHeight(1800);
            elevator.setHookToAttach();
        } else if (gamepad2.dpad_left) {
            elevator.setHeight(1600);
            grabber.slideToPercent(0.15);
        } else if (gamepad2.dpad_down) {
            elevator.setHeight(0);
        }

        elevator.setWenchPower(gamepad2.left_stick_y);

        grabber.run(packet);
        elevator.run(packet);

        packet.put("elevator_at_target", elevator.atTarget());
        packet.put("dt", dt);
        packet.put("slide_pose", slide_position);

        telemetry.addData("x", mecanumDrive.pose.position.x);
        telemetry.addData("y", mecanumDrive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(mecanumDrive.pose.heading.toDouble()));
        telemetry.update();

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), mecanumDrive.pose);

        dash.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {

    }

    private static double linearDeadband(double raw, double deadband) {
        return Math.abs(raw) < deadband ? 0 : Math.signum(raw) * (Math.abs(raw) - deadband) / (1 - deadband);
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static class Params {
        public double slide_speed = 0.0015;
    }
}
