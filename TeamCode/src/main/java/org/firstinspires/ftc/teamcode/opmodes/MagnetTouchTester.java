package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.MagnetAndTouch;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class MagnetTouchTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int Counter = 0;

        CrabRobot robot = new CrabRobot(this);
        waitForStart();
        DigitalChannel touchSensor;  // Touch sensor Object
        touchSensor = robot.getDigitalChannel("touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        while (!isStopRequested())
            robot.update();
            telemetry.update();


            // send the info back to driver station using telemetry function.
            if (touchSensor.getState()) {
                telemetry.addData("Touch Sensor", "Is Pressed");

            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }
            telemetry.addData("Counter",Counter);
            telemetry.addData("State",touchSensor.getState());
            telemetry.addData("Mode",touchSensor.getMode());
            Counter++;


        }

    }

