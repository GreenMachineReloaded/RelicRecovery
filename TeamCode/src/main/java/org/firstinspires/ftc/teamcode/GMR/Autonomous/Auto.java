package org.firstinspires.ftc.teamcode.GMR.Autonomous;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GMR.Robot.Robot;
import org.firstinspires.ftc.teamcode.GMR.Robot.SubSystems.DriveTrain;

public class Auto {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Robot robot;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private NavxMicroNavigationSensor gyroscope;
    private IntegratingGyroscope gyro;

    private Servo leftArm;
    private Servo rightArm;
    private ColorSensor colorSensorLeft;
    private DistanceSensor distanceSensorLeft;
    private ColorSensor colorSensorRight;
    private DistanceSensor distanceSensorRight;

    private ElapsedTime time;

    private State state;
    private StartingLocation startingLocation;

    private boolean isFinished;
    private double currentSeconds;
    private double goalSeconds;
    private StringBuilder stageCheck = new StringBuilder();

    private final double LEFT_INITIAL_POSITION = 0.85;
    private final double RIGHT_INITIAL_POSITION = 0;

    public Auto(StartingLocation startingLocation, HardwareMap hardwareMap, Telemetry telemetry) {
        this.startingLocation = startingLocation;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void init() {
        rightFront = hardwareMap.dcMotor.get("rightfront");
        leftFront = hardwareMap.dcMotor.get("leftfront");
        rightRear = hardwareMap.dcMotor.get("rightrear");
        leftRear = hardwareMap.dcMotor.get("leftrear");

        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorDistanceLeft");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "colorDistanceLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorDistanceRight");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "colorDistanceRight");

        gyroscope = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        robot = new Robot(hardwareMap, telemetry);

        leftArm.setPosition(LEFT_INITIAL_POSITION);
        rightArm.setPosition(RIGHT_INITIAL_POSITION);

        state = State.TIME;
        isFinished = false;

        time = new ElapsedTime();
    }

    public void loop() {
        currentSeconds = time.seconds();
        switch(state) {
            case TIME:
                robot.blockLift.clamp(false, true, true, false);
                stageCheck.append("Time - ");
                state = State.GRAB;
                break;
            case GRAB:
                robot.blockLift.clamp(false, false, false, true);
                goalSeconds = currentSeconds + 0.4;
                stageCheck.append("Grab - ");
                state = State.LIFT;
                break;
            case LIFT:
                if (currentSeconds >= goalSeconds) {
                    switch(startingLocation) {
                        case BLUE_1:
                            robot.blockLift.setLift(500);
                            break;
                        case BLUE_2:
                        case RED_1:
                            robot.blockLift.setLift(400);
                            break;
                        case RED_2:
                            robot.blockLift.setLift(500);
                    }
                    if (startingLocation.isSideOne()) {
                        goalSeconds = currentSeconds + 1;
                    } else {
                        goalSeconds = currentSeconds + 2;
                    }
                    stageCheck.append("Lift - ");
                    state = State.ARMDOWN;
                }
                break;
            case ARMDOWN:
                if (startingLocation.isBlue()) {
                    leftArm.setPosition(0.25);
                } else {
                    rightArm.setPosition(0.5);
                }
                if (currentSeconds >= goalSeconds) {
                    stageCheck.append("Arm Down - ");
                    state = State.READ;
                }
                break;
            case READ:
                int blue;
                int red;
                if (startingLocation.isBlue()) {
                    blue = colorSensorLeft.blue();
                    red = colorSensorLeft.red();
                } else {
                    blue = colorSensorRight.blue();
                    red = colorSensorRight.red();
                }
                telemetry.addData("Blue:", blue);
                telemetry.addData("Red:", red);
                if (blue > red) {
                    state = State.LEFTKNOCK;
                    telemetry.addData("The ball is:", "blue");
                } else {
                    state = State.RIGHTKNOCK;
                    telemetry.addData("The ball is:", "red");
                }
                stageCheck.append("Read - ");
                break;
            case LEFTKNOCK:
                if (!isFinished){
                    isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.S, 0.25, 1);
                } else {
                    isFinished = false;
                    state = State.LEFTARMUP;
                    stageCheck.append("Left Knock - ");
                    time.reset();
                }
                break;
            case RIGHTKNOCK:
                if (!isFinished) {
                    if (startingLocation.isBlue()) {
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.25, 1);
                    } else {
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.25, 0.5);
                    }
                } else {
                    isFinished = false;
                    state = State.RIGHTARMUP;
                    stageCheck.append("Right Knock - ");
                    time.reset();
                }
                break;
            case LEFTARMUP:
            case RIGHTARMUP:
                if (startingLocation.isBlue()) {
                    leftArm.setPosition(LEFT_INITIAL_POSITION);
                } else {
                    rightArm.setPosition(RIGHT_INITIAL_POSITION);
                }
                if (time.seconds() >= 1) {
                    if (state.equals(State.LEFTARMUP)) {
                        state = State.LEFTZONE;
                        stageCheck.append("Left Arm Up - ");
                    } else {
                        state = State.RIGHTZONE;
                        stageCheck.append("Right Arm Up - ");
                    }
                }
                break;
            case LEFTZONE:
                if (!isFinished) {
                    if (startingLocation.isBlue()) {
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 10 );
                    } else if (startingLocation.isSideOne()) {
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 7);
                    } else {
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 2.5);
                    }
                } else {
                    isFinished = false;
                    if (startingLocation.isSideOne()) {
                        state = State.TURNBOX;
                    } else {
                        state = State.STRAFE;
                    }
                    stageCheck.append("Left Zone - ");
                    time.reset();
                }
                break;
            case RIGHTZONE:
                if (!isFinished) {
                    switch(startingLocation) {
                        case BLUE_1:
                            isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 5);
                            break;
                        case BLUE_2:
                            isFinished = isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 2.5);
                            break;
                        case RED_1:
                            isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 12);
                            break;
                        case RED_2:
                            isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.4, 9.5);
                            break;
                    }
                }
        }
    }
}
