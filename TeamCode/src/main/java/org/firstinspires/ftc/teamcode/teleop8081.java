package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "8081 TeleOp - 2024")
public class teleop8081 extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private MecanumDrive mecanumDrive;
    private Elevator elevator;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        elevator = new Elevator(hardwareMap);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // Update pose estimator
        mecanumDrive.updatePoseEstimate();
//        elevator.update(packet);

        // Add driving via controller to the actions list
        runningActions.add(mecanumDrive.controllerDriveAction(gamepad1));
//        runningActions.add(elevator.controllerDriveElevator(gamepad1));



        //////////////////////////////////////////////////
        // Action runner

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }


}
