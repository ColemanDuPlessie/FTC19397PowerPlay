package org.firstinspires.ftc.teamcode.legacy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.Robot19397;
import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;
/*
@Config
@Autonomous(name = "Red Carousel Side Auto (Roadrunner)")
public class RoadrunnerAutoRedCarouselSide extends OpMode {

    public static double STARTXPOS = -36;
    public static double STARTHEADING = 0;
    public static double DUMPXPOS = -45;
    public static double DUMPYPOS = -24;
    public static double DUMPHEADING = 15;
    public static double DUMPDURATION = 5;
    public static double CAROUSELXPOS = -55;
    public static double CAROUSELYPOS = -55;
    public static double CAROUSELHEADING = -90;
    public static double CAROUSELDURATION = 5;
    public static double PARKXPOS = -60;
    public static double PARKYPOS = -36;
    public static double PARKHEADING = 0;

    Robot19397 robot = new Robot19397(new ElapsedTime());
    SampleMecanumDrive drive;
    TrajectorySequence auto;

    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(STARTXPOS, -63, Math.toRadians(STARTHEADING));
        drive.setPoseEstimate(startPose);

        auto = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(DUMPXPOS, DUMPYPOS, Math.toRadians(DUMPHEADING)), Math.toRadians(DUMPHEADING))
                .addDisplacementMarker(() -> {
                    robot.arm.goToPosition(ArmSubsystem.POSITION.UP);
                })
                .waitSeconds(DUMPDURATION)
                .addDisplacementMarker(() -> {
                    robot.arm.goToPosition(ArmSubsystem.POSITION.CLEARTHEBARRIER);
                })
                .splineToSplineHeading(new Pose2d(CAROUSELXPOS, CAROUSELYPOS, Math.toRadians(CAROUSELHEADING)), Math.toRadians(CAROUSELHEADING))
                .addDisplacementMarker(() -> {
                    //robot.setDuckServoPower(1);
                })
                .waitSeconds(CAROUSELDURATION)
                .addDisplacementMarker(() -> {
                    //robot.setDuckServoPower(0);
                })
                .splineToSplineHeading(new Pose2d(PARKXPOS, PARKYPOS, Math.toRadians(PARKHEADING)), Math.toRadians(PARKHEADING))
                .build();
    }

    @Override
    public void start() {
        drive.followTrajectorySequenceAsync(auto);
    }

    @Override
    public void loop() {
        drive.update();
    }

}
*/