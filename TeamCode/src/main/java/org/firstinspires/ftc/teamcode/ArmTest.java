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

    ServoImplEx servoRight;
    ServoImplEx servoLeft;

    ServoImplEx servoSlideRight;
    ServoImplEx servoSlideLeft;

    CRServo intakeRightServo;
    CRServo intakeLeftServo;

    private static final double SERVO_INSIDE = 0.95;
    private static final double SERVO_HOOK = 0.55;
    private static final double SERVO_FLOOR = 0.33;

    private static final double SLIDE_OUT = 0.615;
    private static final double SLIDE_IN = 0.45;

    private double slidePos = SLIDE_IN;






    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servoLeft = hardwareMap.get(ServoImplEx.class, "servoWristLeft");
        servoRight = hardwareMap.get(ServoImplEx.class, "servoWristRight");

        intakeLeftServo = hardwareMap.get(CRServo.class, "servoIntakeLeft");
        intakeRightServo = hardwareMap.get(CRServo.class, "servoIntakeRight");

        servoSlideRight = hardwareMap.get(ServoImplEx.class, "servoSlideRight");
        servoSlideLeft = hardwareMap.get(ServoImplEx.class, "servoSlideLeft");

         servoLeft.setDirection(Servo.Direction.REVERSE);

        servoLeft.setPwmEnable();
        servoRight.setPwmEnable();

        servoSlideLeft.setPwmEnable();
        servoSlideRight.setPwmEnable();

        intakeLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);

        servoSlideLeft.setDirection(Servo.Direction.REVERSE);

        servoSlideLeft.setPosition(PARAMS.slide_goal);
        servoSlideRight.setPosition(PARAMS.slide_goal);


        servoLeft.setPosition(PARAMS.goal);
        servoRight.setPosition(PARAMS.goal);

        intakeLeftServo.setPower(PARAMS.intake_pwr);
        intakeRightServo.setPower(PARAMS.intake_pwr);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.right_bumper) {
            PARAMS.intake_pwr = 1.0;
        } else if (gamepad1.left_bumper) {
            PARAMS.intake_pwr = -1.0;
        } else {
            PARAMS.intake_pwr = 0.0;
        }

        if (gamepad1.x) {
            PARAMS.goal = SERVO_INSIDE;
        } else if (gamepad1.y) {
            PARAMS.goal = SERVO_HOOK;
        } else if (gamepad1.b) {
            PARAMS.goal = SERVO_FLOOR;
        }

        PARAMS.slide_goal = Math.min(Math.max(PARAMS.slide_goal + gamepad1.right_trigger*PARAMS.slide_speed_factor, SLIDE_IN), SLIDE_OUT);
        PARAMS.slide_goal = Math.min(Math.max(PARAMS.slide_goal - gamepad1.left_trigger*PARAMS.slide_speed_factor, SLIDE_IN), SLIDE_OUT);

        servoSlideLeft.setPosition(PARAMS.slide_goal);
        servoSlideRight.setPosition(PARAMS.slide_goal);


        servoLeft.setPosition(PARAMS.goal);
        servoRight.setPosition(PARAMS.goal);

        intakeLeftServo.setPower(PARAMS.intake_pwr);
        intakeRightServo.setPower(PARAMS.intake_pwr);

        packet.put("left_servo", servoLeft.getPosition());
        packet.put("right_servo", servoRight.getPosition());

        dash.sendTelemetryPacket(packet);
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double goal = SERVO_FLOOR;

        public double slide_speed_factor = 0.01;
        public double slide_goal = 0.5;
        public double intake_pwr = 0.0;
    }

}
