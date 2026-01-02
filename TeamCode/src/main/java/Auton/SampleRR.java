package Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import RoadRunner.TankDrive;

@Config
@Autonomous(name = "SampleRR", group = "Autonomous")
@Disabled
public class SampleRR extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        waitForStart();

        if (isStopRequested()) return;

        Action squareDrive = drive.actionBuilder(initialPose)
                .lineToY(24)
                .turnTo(Math.toRadians(0))
                .lineToX(24)
                .turnTo(Math.toRadians(270))
                .lineToY(0)
                .turnTo(Math.toRadians(180))
                .lineToX(0)
                .turnTo(Math.toRadians(90))
                .build();
        Actions.runBlocking(squareDrive);
    }
}