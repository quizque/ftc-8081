package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Elevator Test")
@Config
public class ElevatorTest extends OpMode {

    public static ElevatorTest.Params PARAMS = new ElevatorTest.Params();
    private FtcDashboard dash = FtcDashboard.getInstance();

    private DcMotorEx elevatorLeft;
    private DcMotorEx elevatorRight;

    PIDFController pidLeft;
    PIDFController pidRight;

    private double targetHeight = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevatorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        // Set idle behavior
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motor direction
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidLeft = new PIDFController(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kG);
        pidRight = new PIDFController(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kG);
    }

    long timestamp = System.currentTimeMillis() / 1000;

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.a) {
            targetHeight = 0;
            timestamp = System.currentTimeMillis() / 1000;
        } else if (gamepad1.b) {
            targetHeight = 1000;
                    timestamp = System.currentTimeMillis() / 1000;
        }else if (gamepad1.x) {
            targetHeight = 2500;
                    timestamp = System.currentTimeMillis() / 1000;
        }else if (gamepad1.y) {
            targetHeight = 4000;
                    timestamp = System.currentTimeMillis() / 1000;
        }

        targetHeight = Math.min(Math.max(PARAMS.encoderMinimum, targetHeight), PARAMS.encoderMaximum);
        pidLeft.setP(PARAMS.kP);
        pidLeft.setI(PARAMS.kI);
        pidLeft.setD(PARAMS.kD);
        pidLeft.setF(PARAMS.kG);

        pidRight.setP(PARAMS.kP);
        pidRight.setI(PARAMS.kI);
        pidRight.setD(PARAMS.kD);
        pidRight.setF(PARAMS.kG);

        // Drive motors
        double l = pidLeft.calculate(elevatorLeft.getCurrentPosition(), targetHeight);
        double r = pidRight.calculate(elevatorRight.getCurrentPosition(), targetHeight) ;

        l = Math.min(Math.max(PARAMS.minSpeed, l), PARAMS.maxSpeed);
        r = Math.min(Math.max(PARAMS.minSpeed, r), PARAMS.maxSpeed);

        packet.put("pid_left", l);
        packet.put("pid_right", r);
        packet.put("encoder_left", elevatorLeft.getCurrentPosition());
        packet.put("encoder_right", elevatorRight.getCurrentPosition());
        packet.put("elevator-target", targetHeight);
        elevatorLeft.setPower(l);
        elevatorRight.setPower(r);

        dash.sendTelemetryPacket(packet);
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double kG = 0.1;

        // Normal PID constants
        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.0001;

        public double maxSpeed = 1.0;
        public double minSpeed = -0.3;

        // Elevator height constants (in units of encoder ticks)
        public double encoderMinimum = 0.0;
        public double encoderMaximum = 4000.0;
    }

}
