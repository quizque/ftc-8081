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

    public static ElevatorTestSingleEncoder.Params PARAMS = new ElevatorTestSingleEncoder.Params();
    private FtcDashboard dash = FtcDashboard.getInstance();

    private DcMotorEx elevatorLeft;
    private DcMotorEx elevatorRight;
    private Elevator elevator;

    PIDController pidLeft;
    PIDController pidRight;

    private double targetHeight = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevatorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        // Set idle behavior
//        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motor direction
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pidLeft = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);
        pidRight = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);

    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

//        targetHeight += ;
//        targetHeight -= ;

            elevatorLeft.setPower(gamepad1.right_trigger * PARAMS.contC - gamepad1.left_trigger * PARAMS.contC);
            elevatorRight.setPower(gamepad1.right_trigger * PARAMS.contC - gamepad1.left_trigger * PARAMS.contC);

        targetHeight = Math.min(Math.max(PARAMS.encoderMinimum, targetHeight), PARAMS.encoderMaximum);

        pidLeft.setP(PARAMS.kP);
        pidLeft.setI(PARAMS.kI);
        pidLeft.setD(PARAMS.kD);

        pidRight.setP(PARAMS.kP);
        pidRight.setI(PARAMS.kI);
        pidRight.setD(PARAMS.kD);

        // Drive motors
//        double l = pidLeft.calculate(elevatorLeft.getCurrentPosition(), targetHeight) + PARAMS.kG;
        double r = pidRight.calculate(elevatorRight.getCurrentPosition(), targetHeight) + PARAMS.kG;



//        packet.put("pid_left", l);
        packet.put("pid_right", r);
        packet.put("encoder_left", elevatorLeft.getCurrentPosition());
        packet.put("encoder_right", elevatorRight.getCurrentPosition());
        packet.put("elevator-target", targetHeight);
//        elevatorLeft.setPower(r);
//        elevatorRight.setPower(r);

        dash.sendTelemetryPacket(packet);
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double kG = 0.0;

        // Normal PID constants
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        public double maxSpeed = 0.5;
        public  double minSpeed = 0.1;

        public double contC = 0.5;

        // Elevator height constants (in units of encoder ticks)
        public double encoderMinimum = 0.0;
        public double encoderMaximum = 0.0;
    }

}
