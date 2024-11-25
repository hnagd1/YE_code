package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Methods.attachmentMethods;
import org.firstinspires.ftc.teamcode.Methods.autonomousMethods;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class automounousDrive extends OpMode {

    enum State {
        DRIVE_TO_BLOCKS,
        PICK_UP_BLOCKS,
        DRIVE_TO_DROP_OFF,
        DROP_BLOCKS,
        DONE
    }

    // Tuneing varibles
    int strafeTime = 3;
    int armUpDownTime = 1;
    int initDriveDistance = 20;
    int pickUpBlockTime = 2;

    State state = State.DRIVE_TO_BLOCKS;

    double motorPower = 0.3;

    autonomousMethods drive = new autonomousMethods();
    attachmentMethods arm = new attachmentMethods();


    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();

    public void init() {

        telemetry.addLine("Version: init");

        drive.init(hardwareMap);
        arm.init(hardwareMap);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .build();
    }

    public void init_loop() {

        if (tagProcessor.getDetections().size() > 0) {

            AprilTagDetection tagInit = tagProcessor.getDetections().get(0);

            telemetryAprilTag(tagInit);
        }
    }

    public void loop() {

        AprilTagDetection tag = tagProcessor.getDetections().get(0);
        telemetryAprilTag(tag);

        switch (state) {
            case DRIVE_TO_BLOCKS:

                while (tag.ftcPose.range > initDriveDistance) {
                    if (!tagProcessor.getDetections().isEmpty()) {
                        tag = tagProcessor.getDetections().get(0);
                    }
                    drive.setDriveAngle(0, 0.6);
                }
                drive.stopMotors();
                resetRuntime();

                while (getRuntime() < strafeTime) {
                    drive.setDriveAngle(90, 0.6);
                }
                drive.stopMotors();
                state = State.PICK_UP_BLOCKS;
                break;

            case PICK_UP_BLOCKS:

                resetRuntime();
                while (getRuntime() < armUpDownTime) {
                    arm.toggleLiftArm(-.5, telemetry);
                }
                arm.toggleLiftArm(0, telemetry);
                arm.toggleIntake(1);
                resetRuntime();

                while (getRuntime() < pickUpBlockTime) {
                    drive.setDriveAngle(0, 0.3);
                }
                drive.stopMotors();
                arm.toggleIntake(0);

                while (getRuntime() < armUpDownTime) {
                    arm.toggleLiftArm(-.5, telemetry);
                }
                state = State.DRIVE_TO_DROP_OFF;
                break;

            case DRIVE_TO_DROP_OFF:
                // Code
                state = State.DROP_BLOCKS;
                break;
            case DROP_BLOCKS:
                // Code
                state = State.DONE;
                break;
            default:
                telemetry.addLine("Auto is complete. If this is an error, well your kinda cooked ngl");
        }

    }

    public void telemetryAprilTag(AprilTagDetection tag) {
        telemetry.addData("x", tag.ftcPose.x);
        telemetry.addData("y", tag.ftcPose.y);
        telemetry.addData("z", tag.ftcPose.z);
        telemetry.addData("pitch", tag.ftcPose.pitch);
        telemetry.addData("roll", tag.ftcPose.roll);
        telemetry.addData("yaw", tag.ftcPose.yaw);
        telemetry.addData("range", tag.ftcPose.range);
        telemetry.addData("bearing", tag.ftcPose.bearing);
        telemetry.addData("elevation", tag.ftcPose.elevation);
    }

}
