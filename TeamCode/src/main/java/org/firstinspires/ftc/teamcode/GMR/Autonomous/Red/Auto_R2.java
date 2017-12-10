package org.firstinspires.ftc.teamcode.GMR.Autonomous.Red;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.GMR.Robot.Robot;
import org.firstinspires.ftc.teamcode.GMR.Robot.SubSystems.DriveTrain;

/**
 * Created by FTC 4316 on 11/11/2017
 */
@Autonomous(name = "Auto R2", group = "Red")
public class Auto_R2 extends OpMode {

    private Robot robot;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    NavxMicroNavigationSensor gyroscope;
    IntegratingGyroscope gyro;

    Servo rightArm;
    Servo leftArm;

    ColorSensor colorSensorRight;
    DistanceSensor distanceSensorRight;

    private RedStates state;

    private boolean isFinished;

    private double position;
    private double goalPosition;

    private ElapsedTime time = new ElapsedTime();

    private double currentSeconds;
    private double goalSeconds;

    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("rightfront");
        leftFront = hardwareMap.dcMotor.get("leftfront");
        rightRear = hardwareMap.dcMotor.get("rightrear");
        leftRear = hardwareMap.dcMotor.get("leftrear");

        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm = hardwareMap.get(Servo.class, "leftArm");

        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorDistanceRight");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "colorDistanceRight");

        gyroscope = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        robot = new Robot(hardwareMap, telemetry);

        goalPosition = 0.5;
        position = 0;
        rightArm.setPosition(position); //vertical start
        leftArm.setPosition(0.85);
        // position

        state = RedStates.TIME;
        isFinished = false;

        time.reset();

    }
        @Override
        public void loop(){
        currentSeconds = time.seconds();
            telemetry.addData("State:", state);
            telemetry.update();
            switch(state){
                case TIME:
                    state = RedStates.GRAB;
                    robot.blockLift.clamp(false,true, true, false);
                    break;
                case GRAB:
                    robot.blockLift.clamp(false,false, false, true);
                    state = RedStates.LIFT;
                    goalSeconds = currentSeconds + 0.4;
                case LIFT:
                    if (currentSeconds >= goalSeconds) {
                        robot.blockLift.setLift(700);
                        state = RedStates.ARMDOWN;
                        goalSeconds = currentSeconds += 2.0;
                    }
                case ARMDOWN:
                    //Lowers right arm WORKING
                    rightArm.setPosition(goalPosition);
                    if(currentSeconds >= goalSeconds){
                        state = RedStates.READ; //READ
                    } break;

                case READ:
                    //Reads the color/distance sensor to determine which ball to knock off WORKING
                    if(colorSensorRight.blue() > colorSensorRight.red()){
                        telemetry.addData("Blue:", colorSensorRight.blue());
                        telemetry.addData("Red:", colorSensorRight.red());
                        telemetry.addData("The ball is:", "blue");
                        telemetry.update();

                        state = RedStates.LEFTKNOCK;
                    } else if(colorSensorRight.red() > colorSensorRight.blue()){
                        telemetry.addData("Blue:", colorSensorRight.blue());
                        telemetry.addData("Red:", colorSensorRight.red());
                        telemetry.addData("The ball is:", "red");
                        telemetry.update();

                        state = RedStates.RIGHTKNOCK;
                    } break;

                case LEFTKNOCK:
                    //Knocks the left ball off of the pedestal WORKING
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.25, 1);
                    } else{
                        isFinished = false;
                        state = RedStates.LEFTARMUP;
                        time.reset();
                    } break;

                case RIGHTKNOCK:
                    //Knocks the right ball off of the pedestal WORKING
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.S, 0.25, 0.5);
                    } else{
                        isFinished = false;
                        state = RedStates.RIGHTARMUP;
                        time.reset();
                    } break;

                case LEFTARMUP:
                    //Lifts arm up after knocking left ball WORKING
                    rightArm.setPosition(position);
                    if(time.seconds() >= 1){
                        state = RedStates.LEFTZONE;
                    } break;

                case RIGHTARMUP:
                    //Lifts arm up after knocking right ball WORKING
                    rightArm.setPosition(position);
                    if(time.seconds() >= 1){
                        state = RedStates.RIGHTZONE;
                    } break;

                case LEFTZONE:
                    //Returns to original position from knocking left ball WORKING
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 2.5);
                    } else{
                        isFinished = false;
                        state = RedStates.STRAFE;
                        time.reset();
                    } break;

                case RIGHTZONE:
                    //Returns to original position from knocking right ball WORKING
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 9.5);
                    } else{
                        isFinished = false;
                        state = RedStates.STRAFE;
                        time.reset();
                    } break;

                case STRAFE:
                    //Strafes left to face CryptoBox. UNTESTED/DEACTIVATED
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.W, 0.3, 3.5);
                    } else{
                        isFinished = false;
                        state = RedStates.DRIVEBOX;
                    } break;

                case DRIVEBOX:
                    //Drives into CryptoBox
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.3, 3.5);
                    } else{
                        isFinished = false;
                        state = RedStates.DROP;
                    } break;

                case DROP:
                    robot.blockLift.clamp(false, false,true, false);
                    state = RedStates.DRIVEBACK;
                    break;
                case DRIVEBACK:
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.S, 0.3, 1.5);
                    } else{
                        isFinished = false;
                        state = RedStates.END;
                    } break;
                case END:
                    robot.driveTrain.stop();
                    break;
            }

        }

}

enum RedStates {
    TIME,
    ARMDOWN,
    READ,
    LEFTKNOCK,
    RIGHTKNOCK,
    LEFTARMUP,
    RIGHTARMUP,
    LEFTZONE,
    RIGHTZONE,
    STRAFE,
    DRIVEBOX,
    DRIVEBACK,
    END,
    GRAB,
    DROP,
    LIFT
}

