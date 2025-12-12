package Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import Systems.Robot;

/**
 * Title: BasicAuto - Designed for FTC Decode 2025-26
 */

@Autonomous(name = "AutoRed", group = "Auton")
public class AutonRed1 extends LinearOpMode {

    private final Robot robot = new Robot();

    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double FLYWHEEL_TARGET_RPS    = 52.0;

    private static final double ARTIFACT_HOLD_LEFT     = 0.0;
    private static final double ARTIFACT_RELEASE_LEFT  = 0.5;
    private static final double ARTIFACT_HOLD_RIGHT    = 0.5;
    private static final double ARTIFACT_RELEASE_RIGHT = 1.0;

    private static final double ROTATE_POWER   = 0.3;
    // 278 ms => 0.278 s
    private static final double ROTATE_TIME_S  = 0.228; //0.278

    private static final double DRIVE_POWER    = 0.5;
    // 1000 ms => 1.0 s
    private static final double DRIVE_TIME_S   = 1.0;

    // Motif read timeout (3000 ms => 3.0 s)
    private static final double MOTIF_READ_TIMEOUT_S = 3.0;

    private int motifID = 0;
    private String motifPattern = "UNKNOWN";

    // --------------------------------------------------------
    // NON-BLOCKING WAIT (SECONDS, USING CLOCK)
    // --------------------------------------------------------
    private void waitSec(double sec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < sec) {
            idle();
        }
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        FtcDashboard.getInstance().startCameraStream(robot.vision.limeLight, 90);

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1) Read motif (3 seconds timeout)
        motifID = readMotifFromTag(MOTIF_READ_TIMEOUT_S);
        telemetry.addData("Motif ID", motifID);
        telemetry.addData("Motif Pattern", motifPattern);
        telemetry.update();

        waitSec(2.0);
        driveForwardByTime(1,0.15);
        // 2) Rotate
        rotateLeftByTime(ROTATE_POWER, ROTATE_TIME_S);

        // 3) Shoot
        shootBasedOnMotif(motifID);

        waitSec(5.0);

        // 4) Drive
        driveForwardByTime(DRIVE_POWER, DRIVE_TIME_S);

        stopAll();
    }

    // --------------------------------------------------------
    // APOLLO MOTIF READING (SECONDS, USING ElapsedTime)
    // --------------------------------------------------------
    private int readMotifFromTag(double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        motifPattern = "UNKNOWN";
        motifID = 0;

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            LLResult res = robot.vision.limeLight.getLatestResult();
            boolean hasTarget = res != null && res.isValid();

            telemetry.addData("LimeLight Search", hasTarget ? "Target in View" : "Searching");
            telemetry.update();

            if (hasTarget) {
                List<LLResultTypes.FiducialResult> fiducials = res.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    int id = fiducials.get(0).getFiducialId();

                    switch (id) {
                        case 21: motifPattern = "GPP"; break;
                        case 22: motifPattern = "PGP"; break;
                        case 23: motifPattern = "PPG"; break;
                        default: motifPattern = "UNKNOWN"; break;
                    }

                    motifID = id;
                    return id;
                }
            }

            idle();
        }

        return 0;
    }

    // --------------------------------------------------------
    // DRIVE / ROTATION (SECONDS, USING CLOCK)
    // --------------------------------------------------------
    private void rotateLeftByTime(double power, double durationSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < durationSec) {
            robot.driveTrain.tankDrive(0.0, power);
            idle();
        }
        robot.driveTrain.tankDrive(0.0, 0.0);
    }

    private void driveForwardByTime(double power, double durationSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < durationSec) {
            robot.driveTrain.tankDrive(power, 0.0);
            idle();
        }
        robot.driveTrain.tankDrive(0.0, 0.0);
    }

    // --------------------------------------------------------
    // FLYWHEEL CONTROL
    // --------------------------------------------------------
    private void chargeFlywheel() {
        double targetTicksPerSec = FLYWHEEL_TARGET_RPS * FLYWHEEL_TICKS_PER_REV;

        robot.scoringMechanisms.flyWheel1.setVelocity(targetTicksPerSec);
        robot.scoringMechanisms.flyWheel2.setVelocity(targetTicksPerSec);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 2.0) {
            double measuredRps =
                    Math.abs(robot.scoringMechanisms.flyWheel1.getVelocity()) /
                            FLYWHEEL_TICKS_PER_REV;

            telemetry.addData("Flywheel RPS", measuredRps);
            telemetry.addData("Target RPS", FLYWHEEL_TARGET_RPS);
            telemetry.update();

            if (measuredRps >= (FLYWHEEL_TARGET_RPS - 0.5)) break;
            idle();
        }
    }

    // --------------------------------------------------------
    // SHOOTING
    // --------------------------------------------------------
    private void shootBasedOnMotif(int motif) {

        chargeFlywheel();

        switch (motif) {
            case 21: // GPP
                fireRightOnce();
                fireLeftOnce();
                fireLeftOnce();
                break;

            case 22: // PGP
                fireLeftOnce();
                fireRightOnce();
                fireLeftOnce();
                break;

            case 23: // PPG
                fireLeftOnce();
                fireLeftOnce();
                fireRightOnce();
                break;

            default:
                fireLeftOnce();
                fireRightOnce();
                fireLeftOnce();
                break;
        }

        robot.scoringMechanisms.flyWheel1.setPower(0.0);
        robot.scoringMechanisms.flyWheel2.setPower(0.0);
    }

    // --------------------------------------------------------
    // FIRE SEQUENCES (USING waitSec, NO SLEEP)
    // --------------------------------------------------------
    private void fireLeftOnce() {
        robot.scoringMechanisms.leftRelease.setPosition(ARTIFACT_RELEASE_LEFT);
        waitSec(0.7);
        robot.scoringMechanisms.leftRelease.setPosition(ARTIFACT_HOLD_LEFT);
        waitSec(0.25);

        robot.scoringMechanisms.rollerIntake.setPower(1.0);
        robot.scoringMechanisms.sorterIntake.setPower(1.0);
        waitSec(1.0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
    }

    private void fireRightOnce() {
        robot.scoringMechanisms.rightRelease.setPosition(ARTIFACT_RELEASE_RIGHT);
        waitSec(0.7);
        robot.scoringMechanisms.rightRelease.setPosition(ARTIFACT_HOLD_RIGHT);
        waitSec(0.25);

        robot.scoringMechanisms.rollerIntake.setPower(1.0);
        robot.scoringMechanisms.sorterIntake.setPower(-1.0);
        waitSec(1.0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
    }

    // --------------------------------------------------------
    // STOP EVERYTHING
    // --------------------------------------------------------
    private void stopAll() {
        robot.driveTrain.tankDrive(0.0, 0.0);
        robot.scoringMechanisms.flyWheel1.setPower(0.0);
        robot.scoringMechanisms.flyWheel2.setPower(0.0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
        robot.scoringMechanisms.leftRelease.setPosition(ARTIFACT_HOLD_LEFT);
        robot.scoringMechanisms.rightRelease.setPosition(ARTIFACT_HOLD_RIGHT);

        try {
            robot.vision.limeLight.stop();
        } catch (Exception ignored) {}
    }
}
