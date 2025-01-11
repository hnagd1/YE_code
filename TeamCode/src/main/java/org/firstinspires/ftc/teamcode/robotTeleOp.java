package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Methods.attachmentMethods;
import org.firstinspires.ftc.teamcode.Methods.autonomousMethods;
import org.firstinspires.ftc.teamcode.Methods.drivingMethods;

@TeleOp
public class robotTeleOp extends OpMode {

    double yPressTime;
    boolean yPress;

    double lockArmTime;
    boolean lockArm;
    double bucketDumpTime;
    boolean bucketDump;
    boolean bucketRetract;


    boolean IMUReset;


    drivingMethods drive = new drivingMethods();
    autonomousMethods auto = new autonomousMethods();
    attachmentMethods attachment = new attachmentMethods();
    LinearSlide ls = new LinearSlide();


    @Override
    public void init() {
        //initialize functions with the hardware map
        drive.init(hardwareMap);
        auto.init(hardwareMap);
        attachment.init(hardwareMap);
        ls.init(hardwareMap);

    }

    @Override
    public void loop() {
        //for test:
        robotCentricDrive();
        //call the functions that control the different functions of the robot
        automatedActions(); //Put automated actions before others to prioritize them
        intakeSystem();
        //linearSlides();
    }

    @Override
    public void start() {}

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
        drive.fieldCentric(lx, ly, rx, gamepad1.right_trigger, IMUReset, telemetry);
    }

    public void intakeSystem() {
        //This gives a value that will be 1 when right, -1 when left, and the middle when both.
        double liftArmMov = gamepad2.right_trigger - gamepad2.left_trigger;

        //Checks if the linear slide is raised to prevent the robot from becoming too large
        if (ls.pos() > 2000) {
            attachment.toggleLiftArm(liftArmMov, telemetry);
        }

        //Toggle the lift arm controlled by the input of the triggers
        attachment.jointMovement(gamepad2.dpad_down, gamepad2.dpad_up, telemetry);

        //set intake power to 1 if b is pressed, -1 if x is pressed, and 0 if nothing is pressed.
        if (gamepad2.b) {
            attachment.toggleIntake(1);
        } else if (gamepad2.x) {
            attachment.toggleIntake(-1);
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
            ls.basketPos(0.935);
            attachment.rotateLiftArm(1, -4300, telemetry);
            attachment.setServoPosition(1,0.6);
        }
        /**
         * Move to high basket position:
         * This action sets the sets the intake servo to the arm lock position.
         * After it has given the servo 1 second to move into place, it then moves the intake arm into place and lifts the slides.
         */
        if (gamepad1.a) {
            attachment.setServoPosition(1,0.66);
            lockArmTime = getRuntime();
            lockArm = true;
        }
        if ((getRuntime()-lockArmTime > 1.0)&lockArm) {
            attachment.rotateLiftArm(1, -5600, telemetry);
            ls.runLinearSlide(0.7, telemetry);
            lockArm = false;
        }
        /*Dump Sample and Retract slides*/
        if (gamepad1.b) {
            ls.basketPos(0.75);
            bucketDumpTime = getRuntime();
            bucketDump = true;
        }
        if ((getRuntime()- bucketDumpTime > 0.75) & bucketDump) {
            ls.basketPos(0.935);
            bucketDumpTime = getRuntime();
            bucketRetract = true;
            bucketDump = false;
        }
        if ((getRuntime()- bucketDumpTime > 0.5) & bucketRetract) {
            ls.runLinearSlide(-0.7, telemetry);
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

//jack is dumb fr fr nc bbs