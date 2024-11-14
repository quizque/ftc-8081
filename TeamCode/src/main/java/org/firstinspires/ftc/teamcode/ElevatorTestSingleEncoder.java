package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Elevator Test Single Encoder")
@Config
public class ElevatorTestSingleEncoder extends OpMode {

    public static ElevatorTest.Params PARAMS = new ElevatorTest.Params();
    private FtcDashboard dash = FtcDashboard.getInstance();

    private DcMotorEx elevatorLeft;
    private DcMotorEx elevatorRight;
    private Elevator elevator;

    PIDController pidLeft;
    PIDController pidRight;

    private double targetHeight = 0.0;
    private double targetHeightSmooted = 0.0;

    private TrapezoidalMotionProfile tProfile = new TrapezoidalMotionProfile(0.0, 0.0, 0.0, 0.0);

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

        pidLeft = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);
        pidRight = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);

    }

    long timestamp = System.currentTimeMillis() / 1000;
    double currPos = 0.0;

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        elevatorLeft.setPower(gamepad1.left_stick_y);
        elevatorRight.setPower(gamepad1.left_stick_y);

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
