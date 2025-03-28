package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.linearSlideMethods;
import org.firstinspires.ftc.teamcode.Methods.attachmentMethods;
import org.firstinspires.ftc.teamcode.Methods.autonomousMethods;
import org.firstinspires.ftc.teamcode.Methods.drivingMethods;

@Config
@TeleOp
public class robotTeleOp extends OpMode {

    double lockArmTime;
    boolean lockArm;
    double bucketDumpTime;
    boolean bucketDump;
    boolean bucketRetract;


    boolean IMUReset;


    drivingMethods drive = new drivingMethods();
    autonomousMethods auto = new autonomousMethods();
    attachmentMethods attachment = new attachmentMethods();
    linearSlideMethods ls = new linearSlideMethods();

    // Auto action adjustment varibles, these are used to adjust pos in real time

    public static double BUCKET_INTAKE_POS = 0.9;
    public static int ARM_INTAKE_POS = -2500;
    public static int ARM_HIGH_BASKET_POS = -3800;
    public static double BUCKET_DUMP_POS = 0.333;
    public static double BUCKET_RETRACT_POS = 0.9;

    @Override
    public void init() {
        // Allows the telemetry variable to send data to both DS and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize functions with the hardware map
        drive.init(hardwareMap);
        auto.init(hardwareMap);
        attachment.init(hardwareMap);
        ls.init(hardwareMap);

    }

    @Override
    public void loop() {
        fieldCentricDrive();

        automatedActions(); //Put automated actions before others to prioritize them
        intakeSystem();
        //linearSlides();

        ls.pos(telemetry);
    }

    @Override
    public void start() {
        //unused
    }

    public void unwind() {
        ls.basketPos(0.87);
        attachment.setServoPosition(1,1);
        // TODO: Check to make sure that the position value of this servo is correct
        attachment.setServoPosition(4, 0);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        attachment.rotateLiftArm(1, -3100, telemetry);
        attachment.setServoPosition(4, 0.8);
    }

    public void robotCentricDrive() {
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        drive.drive(lx, ly, rx, gamepad1.right_trigger);
    }

    public void fieldCentricDrive() {
        IMUReset = gamepad1.options;
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        drive.fieldCentric(lx, ly, rx, gamepad1.right_trigger, IMUReset, telemetry, getRuntime());
    }

    public void intakeSystem() {
        //This gives a value that will be 1 when right, -1 when left, and the middle when both.
        double liftArmMov = gamepad2.right_trigger - gamepad2.left_trigger;

        //Checks if the linear slide is raised to prevent the robot from becoming too large
        if (ls.pos(telemetry) < 2000) {
            attachment.toggleLiftArm(liftArmMov, telemetry);
            attachment.jointMovement(gamepad2.dpad_down, gamepad2.dpad_up, telemetry);
        }

        //Toggle the lift arm controlled by the input of the triggers


        //set intake power to 1 if b is pressed, -1 if x is pressed, and 0 if nothing is pressed.
        if (gamepad2.b) {
            attachment.toggleIntake(1);
        } else if (gamepad2.x) {
            attachment.toggleIntake(-0.5);
        } else {
            attachment.toggleIntake(0);
        }
    }

    public void automatedActions() {
        /**
         * Automated actions are combinations of actions the robot can perform on a simple button press.
         * Some may include multiple parts. This is so they are able to perform timed actions.
         * These timed actions require a variable to be set to the runtime, and a activator variable to be set.
         * The next segment will then look for when the correct amount of time has passed and the action is active.
         */

        /**
         * Intake Dump:
         * This action ensures the basket is in position and moves the intake arm into position.
         */
        if (gamepad2.a) {
            ls.basketPos(BUCKET_INTAKE_POS); // TODO: Adjust this
            attachment.rotateLiftArm(1, ARM_INTAKE_POS, telemetry); // TODO: Adjust this
            attachment.setServoPosition(1,0.066);
        }
        /**
         * Move to high basket position:
         * This action sets the sets the intake servo to the arm lock position.
         * After it has given the servo 1 second to move into place, it then moves the intake arm into place and lifts the slides.
         */
        if (gamepad1.a) {
            attachment.setServoPosition(1,0.075);
            attachment.rotateLiftArm(1, ARM_HIGH_BASKET_POS, telemetry);
            lockArmTime = getRuntime();
            lockArm = true;
        }
        if ((getRuntime()-lockArmTime > 0.25)&lockArm) {
             // TODO: Adjust this
            ls.runLinearSlide(1.0, telemetry);
            lockArm = false;
        }
        /*Dump Sample and Retract slides*/
        if (gamepad1.b) {
            ls.basketPos(BUCKET_DUMP_POS); // TODO: Adjust this
            bucketDumpTime = getRuntime();
            bucketDump = true;
        }
        if ((getRuntime()- bucketDumpTime > 0.8) & bucketDump) {
            ls.basketPos(BUCKET_RETRACT_POS); // TODO: Adjust this
            bucketDumpTime = getRuntime();
            bucketRetract = true;
            bucketDump = false;
        }
        if ((getRuntime()- bucketDumpTime > 0.5) & bucketRetract) {
            ls.runLinearSlide(-1.0, telemetry);
            bucketRetract = false;
        }

    }

    public void linearSlides() {
        ls.loop(gamepad2.left_bumper, gamepad2.right_bumper, telemetry);
    }

    public void stop() { //this needs to stop everything
        drive.setMode(true); //this sets the motors to brake
        drive.setPower(0,0,0,0); //Set motor powers to 0
        ls.stop(); //stops the linear slides
    }
}