package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class Intake implements Subsystem {
    //Hardware: 1 motor, 2 servo
    double syncFactor = 1.05;
    private DcMotorEx intakeMotor;
    private DistanceSensor intakeTopSensor;
    private DistanceSensor intakeBackSensor;
    private double motorPosition = 0;
    private Servo intakeServoL;
    private Servo intakeServoR;
    private double baseposl = 0.217;
    private double baseposr = 0.76715;
    private double intakeposL = 0.814;
    private double intakeposR = 0.1403;
    //placeholder outtake position, may change depending on outtake
    private double outtakeposL = 0.129;
    private double outtakeposR = 0.8595;
    private double lowerlimitL = 0.814;
    private double lowerlimitR = 0.13;
    private double upperlimitL = 0.09;
    private double upperlimitR = 0.9;
    public Intake(Robot robot) {
        //intakeMotor = robot.getMotor("intakeMotor");
        //intakeServoL = robot.getServo("intakeServoL");
        //intakeServoR = robot.getServo("intakeServoR");
    //   intakeCSensor = robot.getColorRangeSensor("intakeCSensor");
        intakeBackSensor = robot.getDistanceSensor("intakeBackSensor");
        intakeTopSensor = robot.getDistanceSensor("intakeTopSensor");
    }
    public void reset(){
      //  intakeServoL.setPosition(baseposl);
      //  intakeServoR.setPosition(baseposr);
    }
    public void intakepos(){
        //intakeServoR.setPosition(intakeposR);
        //intakeServoL.setPosition(intakeposL);
    }
    public void outtakepos(){
       // intakeServoL.setPosition(outtakeposL);
       // intakeServoR.setPosition(outtakeposR);
    }

   /* public void moveArm(double d){
        double targetPosR = intakeServoR.getPosition()+(0.01*d*syncFactor);
        if(targetPosR>lowerlimitR&&targetPosR<upperlimitR) {
            intakeServoL.setPosition(intakeServoL.getPosition() + (0.01 * -d)); //2 degrees??
            intakeServoR.setPosition(intakeServoR.getPosition() + (0.01 * d * syncFactor));
        }
    } */
    /*public double getRightServoPos() {
        return intakeServoR.getPosition();
    }
    public double getLeftServoPos(){
      //  return intakeServoL.getPosition();
    }*/
    public double getDistanceSensed(){
        return intakeTopSensor.getDistance(DistanceUnit.MM);
    }
    public double getBackDistance(){
        //return intakeCSensor.getDistance(DistanceUnit.MM);
        return intakeBackSensor.getDistance(DistanceUnit.MM);
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
