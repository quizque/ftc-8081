package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class LeftSideAuto extends LinearOpMode {
    private Elevator elevator;
    private Grabber grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        elevator = new Elevator(hardwareMap);
        grabber = new Grabber(hardwareMap);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-39.56, -62.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-47.83, -47.83), Math.toRadians(45.00))
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-48.03, -48.03, Math.toRadians(45.00)))
                .lineToConstantHeading(new Vector2d(-60.80, -58.84))
                .build();

        drive.setPoseEstimate(trajectory0.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajectory0);
        elevator.setHeight(4000);
        while (!elevator.atTarget()) {}
        drive.followTrajectorySequence(traj1);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
