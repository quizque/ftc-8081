package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Elevator {
    public static Elevator.Params PARAMS = new Elevator.Params();
//    private FtcDashboard dash = FtcDashboard.getInstance();

    private final DcMotorEx elevatorLeft;
    private final DcMotorEx elevatorRight;

    private final PIDFController pidLeft;
    private final PIDFController pidRight;

    private double targetHeight = 0.0;

    private final Timing.Timer resetEncoderTimer;

    public Elevator(HardwareMap hardwareMap) {

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

        pidLeft.setP(PARAMS.kP);
        pidLeft.setI(PARAMS.kI);
        pidLeft.setD(PARAMS.kD);

        pidRight.setP(PARAMS.kP);
        pidRight.setI(PARAMS.kI);
        pidRight.setD(PARAMS.kD);

        resetEncoderTimer = new Timing.Timer(3);
    }

    public void setHeight(double height) {
        targetHeight = Math.min(Math.max(PARAMS.encoderMinimum, height), PARAMS.encoderMaximum);
    }

    public boolean atTarget() {
        return Math.abs(((elevatorLeft.getCurrentPosition() + elevatorRight.getCurrentPosition()) / 2.0) - targetHeight) < 80.0;
    }
    public boolean atTarget(double error) {
        return Math.abs(((elevatorLeft.getCurrentPosition() + elevatorRight.getCurrentPosition()) / 2.0) - targetHeight) < error;
    }

    public void run() {
//        TelemetryPacket packet = new TelemetryPacket();

        // Drive motors
        double l = pidLeft.calculate(elevatorLeft.getCurrentPosition(), targetHeight) + PARAMS.kG;
        double r = pidRight.calculate(elevatorRight.getCurrentPosition(), targetHeight) + PARAMS.kG;

        l = Math.min(Math.max(PARAMS.minSpeed, l), PARAMS.maxSpeed);
        r = Math.min(Math.max(PARAMS.minSpeed, r), PARAMS.maxSpeed);

        if (targetHeight == 0.0 && elevatorLeft.getCurrentPosition() <= 100 && elevatorRight.getCurrentPosition() <= 100) {
            l = 0;
            r = 0;

            if (!resetEncoderTimer.isTimerOn()) {
                resetEncoderTimer.start();

            }

            if (resetEncoderTimer.done()) {
                elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                resetEncoderTimer.pause();
            }
        } else {
            resetEncoderTimer.pause();
        }

//        packet.put("pid_left", l);
//        packet.put("pid_right", r);
//        packet.put("encoder_left", elevatorLeft.getCurrentPosition());
//        packet.put("encoder_right", elevatorRight.getCurrentPosition());
//        packet.put("elevator-target", targetHeight);
//        packet.put("reset_timer", resetEncoderTimer.remainingTime());

        elevatorLeft.setPower(l);
        elevatorRight.setPower(r);

//        dash.sendTelemetryPacket(packet);
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double kG = 0.125;

        // Normal PID constants
        public double kP = 0.01;
        public double kI = 0.0;
        public double kD = 0.0001;

        public double maxSpeed = 1.0;
        public double minSpeed = -0.3;

        // Elevator height constants (in units of encoder ticks)
        public double encoderMinimum = 0.0;
        public double encoderMaximum = 4000.0;
    }
}
