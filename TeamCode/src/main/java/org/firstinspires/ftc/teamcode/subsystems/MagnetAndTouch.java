package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class MagnetAndTouch implements Subsystem {
    //Hardware: 1 motor, 2 servo
    //private TouchSensor slideTouchSensor;
    //private TouchSensor magnetLimitSwitch;
    public MagnetAndTouch(Robot robot) {
        //intakeMotor = robot.getMotor("intakeMotor");
        //intakeServoL = robot.getServo("intakeServoL");
        //intakeServoR = robot.getServo("intakeServoR");
        //   intakeCSensor = robot.getColorRangeSensor("intakeCSensor");
        //slideTouchSensor = robot.getTouchSensor("touchSensor");
        //magnetLimitSwitch = robot.getTouchSensor("limitSwitch");
    }


    /*public boolean givemagState(){
        return magnetLimitSwitch.isPressed();
    }
    public double givemagVal(){
        return magnetLimitSwitch.getValue();
    }
    public boolean givetouchState(){
        return slideTouchSensor.isPressed();
    }
    public double givetouchVal(){
        return slideTouchSensor.getValue();
    }

 /*   public double getRed(){
        return intakeCSensor.red();
    }


    public double getCProximity(){
        return intakeCSensor.get
    }
    public double getblue(){
        return intakeCSensor.blue();
    }
    public double getGreen(){
        return intakeCSensor.green();
    }
*/

    /*public void setPower(double power) {
        this.motorPosition = power;


        // set encode to new position
    }*/


    @Override
    public void update(TelemetryPacket packet) {
        // intakeMotor.setPower(motorPosition);
    }
}
