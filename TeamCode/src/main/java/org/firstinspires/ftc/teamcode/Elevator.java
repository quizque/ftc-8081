package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Elevator {
    public static Elevator.Params PARAMS = new Elevator.Params();
    private final DcMotorEx elevatorLeft, elevatorRight;

    // For the PID
    private double previousErrorLeft, previousErrorRight;
    private double integralErrorLeft, integralErrorRight;
    private double prevTime;
    private final ElapsedTime runtime;

    private int targetHeight;


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

        runtime = new ElapsedTime();
    }

    public void update() {
        // Get current time
        double currentTime = runtime.time();
        double dt = currentTime - prevTime;

        // Get encoder values
        double encoderLeft = (double)elevatorLeft.getCurrentPosition();
        double encoderRight = (double)elevatorRight.getCurrentPosition();

        // Calculate feedforward
        double ffLeft = PARAMS.kG + PARAMS.kS * Math.signum(elevatorLeft.getVelocity()) + PARAMS.kV * elevatorLeft.getVelocity();
        double ffRight = PARAMS.kG + PARAMS.kS * Math.signum(elevatorRight.getVelocity()) + PARAMS.kV * elevatorRight.getVelocity();

        // Calculate error
        double errorLeft = encoderLeft - targetHeight;
        double errorRight = encoderRight - targetHeight;

        // Calculate feedback
        integralErrorLeft += errorLeft;
        integralErrorLeft = Math.min(Math.max(integralErrorLeft, -PARAMS.kIMaxRange), PARAMS.kIMaxRange); // Prevent spool up
        integralErrorRight += errorRight;
        integralErrorRight = Math.min(Math.max(integralErrorRight, -PARAMS.kIMaxRange), PARAMS.kIMaxRange); // Prevent spool up

        double fbLeft = PARAMS.kP * errorLeft + PARAMS.kI * integralErrorLeft * dt + PARAMS.kD * (errorLeft - previousErrorLeft) * dt;
        double fbRight = PARAMS.kP * errorRight + PARAMS.kI * integralErrorRight * dt + PARAMS.kD * (errorRight - previousErrorRight) * dt;


        // Drive motors
        elevatorLeft.setPower(ffLeft + fbLeft);
        elevatorRight.setPower(fbRight + fbRight);


        // Store for use during next loop
        previousErrorLeft = errorLeft;
        previousErrorRight = errorRight;

        prevTime = currentTime;
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double kG = 0.0;
        public double kS = 0.0;
        public double kV = 0.0;
//        public double kA = 0.0; // Not implemented

        // Normal PID constants
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        // I anti-spool up (this will allow a max of -x to +x)
        public double kIMaxRange = 0.0;

        // Elevator height constants (in units of encoder ticks)
        public double encoderMinimum = 0.0;
        public double encoderMaximum = 0.0;
    }
}
