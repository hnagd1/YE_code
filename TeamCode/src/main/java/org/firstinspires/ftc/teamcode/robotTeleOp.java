package org.firstinspires.ftc.teamcode;import com.qualcomm.robotcore.eventloop.opmode.OpMode;import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import org.firstinspires.ftc.teamcode.Methods.attachmentMethods;import org.firstinspires.ftc.teamcode.Methods.autonomousMethods;import org.firstinspires.ftc.teamcode.Methods.drivingMethods;



import org.firstinspires.ftc.robotcore.external.Telemetry;




@TeleOp
public class robotTeleOp extends OpMode {

    // Removable codeee
    boolean armLifted;
    boolean intakeOn;

    double yPressTime;
    boolean yPress;
    double aPressTime;
    boolean aPress;
    double slideContractTime;
    boolean slideContract;
    boolean brakeButton = false;
    boolean rSB = false;

    //TODO: add comments on this
    drivingMethods drive = new drivingMethods();
    autonomousMethods auto = new autonomousMethods();
    attachmentMethods attachment = new attachmentMethods();
    LinearSlide ls = new LinearSlide();


    @Override
    public void init() { //TODO: add comments on this
        drive.init(hardwareMap);
        auto.init(hardwareMap);
        attachment.init(hardwareMap);
        ls.init(hardwareMap);
        telemetry.addData("Version", 1);

        // TODO: Port this over to auto
        auto.setDriveAngle(0, 0.4);
        try {
            Thread.sleep(650);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        auto.stopMotors();
        ls.extendLinearSlide(telemetry);
        try {
            Thread.sleep(1200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        attachment.rotateLiftArm(1,2000,telemetry);
    }

    public void start() {
        ls.init(hardwareMap);
    } //TODO: figure out why this exists.

    @Override
    public void loop() {

        //TODO: Port this over to the functions (And make sure It works)
        /*double lx = gamepad1.left_stick_x * 1.1; //Strafe variable
        double rx = -gamepad1.left_stick_y; //Forward and backward
        double ly = gamepad1.right_stick_x; //Turn right and left
        boolean turboButton = gamepad1.a;
        drive.drive(rx,lx,ly,turboButton);*/


        //call the functions that control the different functions of the robot
        automatedActions(); //Put automated actions before others to prioritize them
        intakeArm();
        linearSlides();
    }

    public void normalDrive() {
        //TODO: insert normal drive code into here
    }

    public void fieldCentricDrive() {
        //TODO: insert field centric code into here
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

    public void automatedActions() { //TODO: add comments on this
        if (gamepad2.y) {
            attachment.rotateLiftArm(1,1276,telemetry);
            attachment.toggleIntake(-0.2);
            yPressTime = getRuntime();
            yPress = true;
        }

        if ((getRuntime()-yPressTime>1)&yPress) {
            attachment.toggleIntake(0);
            yPress = false;
        }

        if (gamepad2.a) {
            ls.basketPos(0);
            aPressTime = getRuntime();
            aPress = true;
        }

        if ((getRuntime()-aPressTime>0.8)&aPress) {
            aPress = false;
            ls.basketPos(1);
            ls.contractLinearSlide(telemetry);
            slideContractTime = getRuntime();
            slideContract = true;
        }

        if ((getRuntime()-slideContractTime>1.2)&slideContract) {
            slideContract = false;
            ls.basketPos(0.85);
        }
        telemetry.addData("slideContractTime", (getRuntime()-slideContractTime));
        telemetry.addData("slideContract", slideContract);
    }

    public void linearSlides() { //TODO: add comments on this
        ls.loop(gamepad2.left_bumper, gamepad2.right_bumper, !slideContract, telemetry);
    }

    public void stop() { //TODO: add comments on this
        drive.setMode(true);
        attachment.resetServo();
        drive.setPower(0,0,0,0);
    }
}


