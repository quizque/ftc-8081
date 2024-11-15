package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;
import java.util.TimerTask;

@TeleOp(name = "Elevator Test")
@Config
public class ElevatorTest extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();


    public static Elevator elevator;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevator = new Elevator(hardwareMap);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.a) {
            elevator.setHeight(0);
        } else if (gamepad1.b) {
            elevator.setHeight(1000);
        } else if (gamepad1.x) {
            elevator.setHeight(2500);
        } else if (gamepad1.y) {
            elevator.setHeight(4000);
        }

        elevator.run();

        packet.put("elevator_at_target", elevator.atTarget());

        dash.sendTelemetryPacket(packet);
    }
}