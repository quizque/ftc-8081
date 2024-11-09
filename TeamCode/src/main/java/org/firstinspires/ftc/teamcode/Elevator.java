package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Elevator {
    public static Elevator.Params PARAMS = new Elevator.Params();
    private final DcMotorEx elevatorLeft, elevatorRight;

    PIDController pidLeft;
    PIDController pidRight;

    private double targetHeight;


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

    }

    public void update(TelemetryPacket packet) {

        pidLeft.setP(PARAMS.kP);
        pidLeft.setI(PARAMS.kI);
        pidLeft.setD(PARAMS.kD);

        pidRight.setP(PARAMS.kP);
        pidRight.setI(PARAMS.kI);
        pidRight.setD(PARAMS.kD);

        // Drive motors
        double l = pidLeft.calculate(elevatorLeft.getCurrentPosition(), targetHeight) + PARAMS.kG;
        double r = pidRight.calculate(elevatorRight.getCurrentPosition(), targetHeight) + PARAMS.kG;
        packet.put("pid_left", l);
        packet.put("pid_right", r);
        elevatorLeft.setPower(l);
        elevatorRight.setPower(r);
    }

    public class ControllerDriveElevator implements Action {

        Gamepad gamepad;

        public ControllerDriveElevator(Gamepad gamepad) {
            this.gamepad = gamepad;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {


            targetHeight += (float) (gamepad.right_trigger * 0.2);
            targetHeight -= (float) (gamepad.left_trigger * 0.2);

//            elevatorLeft.setPower(gamepad.right_trigger + PARAMS.kG);
//            elevatorRight.setPower(gamepad.right_trigger + PARAMS.kG);

            targetHeight = Math.min(Math.max(PARAMS.encoderMinimum, targetHeight), PARAMS.encoderMaximum);


            packet.put("target_height", targetHeight);
            packet.put("elevator_power", gamepad.right_trigger);
            packet.put("left_encoder", elevatorLeft.getCurrentPosition());
            packet.put("right_encoder", elevatorRight.getCurrentPosition());
//            packet.put("angle_power", rotation_power);

            return true;
        }
    }

    public Action controllerDriveElevator(Gamepad gamepad) {
        return new Elevator.ControllerDriveElevator(gamepad);
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double kG = 0.0;

        // Normal PID constants
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        // Elevator height constants (in units of encoder ticks)
        public double encoderMinimum = 0.0;
        public double encoderMaximum = 0.0;
        public float controlConst;
    }
}
