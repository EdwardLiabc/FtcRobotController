package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp
public class AATele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        waitForStart();

        while (!isStopRequested()) {
            //EDWARD'S INTAKE
            //TODO1: change intake buttons, current buttons are just for testing purposes
            //telemetry.addData("left_stick_y:", gamepad1.left_stick_y);
            telemetry.update();
            robot.update();
            boolean buttonA = gamepad2.a;
            boolean buttonB = gamepad2.b;
            boolean buttonX = gamepad2.x;
            boolean buttonY = gamepad2.y;
            boolean bumpR = gamepad2.right_bumper;
            if(buttonA) {
                robot.intake.moveArm(0.1);
                robot.intake.setPower(0);
            }
            if(buttonB) {
                robot.intake.moveArm(-0.1);
                robot.intake.setPower(0);
            }
            if(buttonX) {
                robot.intake.reset();
                robot.intake.setPower(0);
            }
            if(buttonY) {
                robot.intake.intakepos();
                robot.intake.setPower(1.0);
            }
            if(bumpR){
                robot.intake.outtakepos();
                robot.intake.setPower(1.0);
            }


            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x));


            //UG OUTTAKE
            boolean dpadUp = gamepad1.dpad_up;//big +
            boolean dpadRight = gamepad1.dpad_right;//small +
            boolean dpadDown = gamepad1.dpad_down;//big -
            boolean dpadLeft = gamepad1.dpad_left;//small -

            if(dpadUp){
                robot.outtake.moveArm(0.1);
            } if(dpadDown) {
                robot.outtake.moveArm(-0.1);
            } if(dpadRight){
                robot.outtake.moveR(0.1);
            } if(dpadLeft) {
                robot.outtake.moveR(-0.1);
            } if (gamepad1.x){
                robot.outtake.resetPos();
            }

            telemetry.addData("right servo position: ", robot.outtake.getRightServoPos());
            telemetry.addData("left servo position: ", robot.outtake.getLeftServoPos());
            telemetry.addData("right intake servo position: ", robot.intake.getRightServoPos());
            telemetry.addData("left intake servo position: ", robot.intake.getLeftServoPos());
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());

        }
    }
}
