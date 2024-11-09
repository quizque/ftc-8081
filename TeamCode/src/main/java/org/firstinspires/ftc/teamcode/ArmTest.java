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

    

    private static final double SERVO_INSIDE = 0.95;
    private static final double SERVO_HOOK = 0.55;
    private static final double SERVO_FLOOR = 0.33;


    private double servoTarget = 0.95;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

         servoLeft = hardwareMap.get(ServoImplEx.class, "servoWristLeft");
         servoRight = hardwareMap.get(ServoImplEx.class, "servoWristRight");

         servoLeft.setDirection(Servo.Direction.REVERSE);

        servoLeft.setPwmEnable();
        servoRight.setPwmEnable();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();



        if (gamepad1.x) {
            PARAMS.goal = SERVO_INSIDE;
        } else if (gamepad1.y) {
            PARAMS.goal = SERVO_HOOK;
        } else if (gamepad1.b) {
            PARAMS.goal = SERVO_FLOOR;
        }

        servoLeft.setPosition(PARAMS.goal);
        servoRight.setPosition(PARAMS.goal);

        packet.put("left_servo", servoLeft.getPosition());
        packet.put("right_servo", servoRight.getPosition());

        dash.sendTelemetryPacket(packet);
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double goal = 0.1;
    }

}
