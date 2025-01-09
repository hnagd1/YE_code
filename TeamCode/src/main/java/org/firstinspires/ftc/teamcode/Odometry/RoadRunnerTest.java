package org.firstinspires.ftc.teamcode.Odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous
public class RoadRunnerTest extends LinearOpMode {
    public class ls {
        private DcMotor ls0;
        private DcMotor ls1;

        public ls(HardwareMap hardwareMap) {
            ls0 = hardwareMap.get(DcMotor.class, "Motor5");
            ls1 = hardwareMap.get(DcMotor.class, "Motor6");
            ls0.setDirection(DcMotorSimple.Direction.FORWARD);
            ls1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class retract implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ls0.setTargetPosition(0);
                ls1.setTargetPosition(0);
                ls0.setPower(0.7);
                ls1.setPower(0.7);
                if (ls0.getCurrentPosition() < 10) {
                    ls0.setPower(0);
                    ls1.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }

        public Action retract(){
            return new retract();
        }

        public class extend implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ls0.setTargetPosition(3100);
                ls1.setTargetPosition(3100);
                ls0.setPower(0.7);
                ls1.setPower(0.7);
                if (ls0.getCurrentPosition() > 3050) {
                    ls0.setPower(0);
                    ls1.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }

        public Action extend(){
            return new extend();
        }
    }



    @Override
    public void runOpMode() {
        ls ls = new ls(hardwareMap);

        //TODO: Figure out stop code here

        waitForStart();

        Actions.runBlocking(ls.extend());
    }
}
