package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Methods.attachmentMethods;
import org.firstinspires.ftc.teamcode.Methods.autonomousMethods;
import org.firstinspires.ftc.teamcode.Methods.drivingMethods;
import org.firstinspires.ftc.robotcore.external.Telemetry;




@TeleOp
public class robotTeleOp extends OpMode {

    double yPressTime;
    boolean yPress;
    double aPressTime;
    boolean aPress;
    double slideContractTime;
    boolean slideContract;
    boolean brakeButton = false;
    boolean rSB = false;

    boolean IMUReset;


    //TODO: add comments on this
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

        // TODO: Port this over to auto
        auto.setDriveAngle(0, 0.4); //TODO: make this strafe the correct direction
        try { //wait 650 millis
            Thread.sleep(650);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        auto.stopMotors(); //stops the movement
        ls.extendLinearSlide(telemetry); //extend the slides
        try { //sleep
            Thread.sleep(1200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        attachment.rotateLiftArm(1,2000,telemetry); // rotate lift arm into position.
    }

    public void start() {
        ls.init(hardwareMap);
    } //TODO: figure out why this exists.

    @Override
    public void loop() {
        //for test:
        fieldCentricDrive();
        //call the functions that control the different functions of the robot
        automatedActions(); //Put automated actions before others to prioritize them
        intakeArm();
        linearSlides();
    }

    public void robotCentricDrive() {
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        drive.drive(lx, ly, rx);
    }

    public void fieldCentricDrive() {
        //TODO: insert field centric code into here
        IMUReset = gamepad1.options;
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        drive.fieldCentric(lx, ly, rx, IMUReset);
    }

    public void intakeArm() {
        //This gives a value that will be 1 when right, -1 when left, and the middle when both.
        double liftArmMov = gamepad2.right_trigger - gamepad2.left_trigger;
        //Toggle the lift arm controlled by the input of the triggers
        attachment.toggleLiftArm(liftArmMov, telemetry);

        if (!yPress) { //if the automated action bound to Y is active, this doesn't run

            //set intake power to 1 if b is pressed, -1 if x is pressed, and 0 if nothing is pressed.
            if (gamepad2.b) {
                attachment.toggleIntake(1);
            } else if (gamepad2.x) {
                attachment.toggleIntake(-1);
            } else {
                attachment.toggleIntake(0);
            }

        }
    }

    public void automatedActions() {
        //ACTION Y
        if (gamepad2.y) { //When Y is pressed, we want it to go to the correct position, and dump the contents
            attachment.rotateLiftArm(1,1276,telemetry); //this takes it to the correct position (calibrated with init pos)
            attachment.toggleIntake(-0.2); //dumps
            yPressTime = getRuntime(); //saves the runtime at the time of press
            yPress = true;
        }
        //we need to give the arm a second to go into place before dumping
        if ((getRuntime()-yPressTime>1)&yPress) { //this checks if its been more than 1 second since y has been pressed
            attachment.toggleIntake(0); //stop intake
            yPress = false; //this is so this doesn't run again
        }

        //ACTION A
        if (gamepad2.a) { //when a is pressed, we want to dump the basket
            ls.basketPos(0);
            aPressTime = getRuntime();
            aPress = true;
        }
        //we want to give the basket around 0.8 seconds to dump before retracing the slides
        if ((getRuntime()-aPressTime>0.8)&aPress) {
            aPress = false; //this is so this doesn't run again
            ls.basketPos(1);
            ls.contractLinearSlide(telemetry); //TODO: Make sure this works after linear slide fix
            slideContractTime = getRuntime();
            slideContract = true; //make sure the next actions can happen
        }

        if ((getRuntime()-slideContractTime>1.2)&slideContract) {
            slideContract = false; //this is so this doesn't run again
            ls.basketPos(0.85); //get the basket into place after dropping
        }
    }

    public void linearSlides() {
        //This calls the ls loop with all of the required parameters
        //slide contract is required so it doesn't cancel out the automated actions.
        //Without it, this will set motor powers to 0 when they have to be at 1
        ls.loop(gamepad2.left_bumper, gamepad2.right_bumper, !slideContract, telemetry);
    }

    public void stop() { //this needs to stop everything
        drive.setMode(true); //this sets the motors to brake
        attachment.resetServo(); //reset the servos
        drive.setPower(0,0,0,0); //Set motor powers to 0
    }
}


