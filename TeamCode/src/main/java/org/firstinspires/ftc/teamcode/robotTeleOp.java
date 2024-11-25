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

    drivingMethods drive = new drivingMethods();
    autonomousMethods auto = new autonomousMethods();
    attachmentMethods attachment = new attachmentMethods();
    LinearSlide ls = new LinearSlide();


    @Override
    public void init() {
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
    }

    @Override
    public void loop() {
        double lx = gamepad1.left_stick_x * 1.1; //Strafe variable
        double rx = -gamepad1.left_stick_y; //Forward and backward
        double ly = gamepad1.right_stick_x; //Turn right and left
        boolean turboButton = gamepad1.a;

        drive.drive(rx,lx,ly,turboButton);

        if (!yPress) {
            if (gamepad2.b) {
                // auto.setAngle(90, 25,3000);
                attachment.toggleIntake(1);
            } else if (gamepad2.x) {
                attachment.toggleIntake(-1);
            } else {
                attachment.toggleIntake(0);
            }
        }

        double liftArmMov = gamepad2.right_trigger - gamepad2.left_trigger;
        attachment.toggleLiftArm(liftArmMov, telemetry);

        ls.loop(gamepad2.left_bumper, gamepad2.right_bumper, !slideContract, telemetry);


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

    public void stop() {
        drive.setMode(true);
        attachment.resetServo();
        drive.setPower(0,0,0,0);
    }
}


