package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp
@Config
public class RobofestMain extends LinearOpMode {
    /** @noinspection FieldCanBeLocal*/
    private Follower follower;
    private AlphaDisplay display;
    private final Timer stateTime = new Timer();
    private int state = -1;
    private int end = 1;
    private int oldState = -1;
    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.6);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        display = hardwareMap.get(AlphaDisplay.class, "display");

        TouchSensor button = hardwareMap.get(TouchSensor.class, "button");
        boolean oldPressed = false;

        //noinspection unused
        Pose startPoseEast = new Pose(5.5, 14, Math.toRadians(0));
        //noinspection unused
        Pose startPoseWest = new Pose(5.5, 14, Math.toRadians(180));
        //noinspection unused
        Pose startPoseNorth = new Pose( 6.5, 15, Math.toRadians(90));
        //noinspection unused
        Pose startPoseSouth = new Pose(5.5, 14.5, Math.toRadians(-90));
        //noinspection unused
        Pose boxApose = new Pose (8,12, Math.toRadians(-90));
        //noinspection unused
        Pose boxBpose = new Pose (20, 12, Math.toRadians(-90));
        //noinspection unused
        Pose boxCpose = new Pose(30.5, 12, Math.toRadians(-90));
        //noinspection unused
        Pose boxDpose = new Pose (43,12, Math.toRadians(-90));
        //noinspection unused
        Pose boxEpose = new Pose (66.25,11.25, Math.toRadians(-80));

        //noinspection unused
        Pose white1Pose = new Pose(13, 17.5, Math.toRadians(90));
        //noinspection unused
        Pose white2Pose = new Pose(25, 17.5, Math.toRadians(90));
        Pose blackDropPose = new Pose(47.75,23, Math.toRadians(180));
        Pose medalDropPose = new Pose(49,23, Math.toRadians(180));

        Pose crossPose = new Pose(55, 20, Math.toRadians(0));
        Pose legoSouth = new Pose(55, 11, Math.toRadians(-90));
        Pose legoEast = new Pose(62, 16, Math.toRadians(0));
        Pose legoNorth = new Pose(55, 19, Math.toRadians(90));
        Pose medalPose = new Pose(65.25, 15, Math.toRadians(0));

        // ==================================
        // CHANGE THIS STUFF!
        // ==================================

        //noinspection UnnecessaryLocalVariable
        Pose startPose = startPoseWest;
        //noinspection UnnecessaryLocalVariable
        Pose whitePose = white1Pose;
        Pose stackPose = new Pose(boxBpose.getX(), boxBpose.getY(), boxBpose.getHeading());
        Pose blackPose = new Pose(boxCpose.getX(), boxCpose.getY()-1, boxCpose.getHeading());
//        Uncomment lines in white box if black box is E. Also change case 30.

        follower.setStartingPose(startPose);

        PathChain whiteBox = follower.pathBuilder()
            .addPath(new BezierLine(startPose, whitePose.plus(new Pose (0,-3))))
            .setLinearHeadingInterpolation(startPose.getHeading(),whitePose.getHeading())
            .addPath(new BezierLine(whitePose, whitePose.plus(new Pose (0,-3))))
            .build();

        changeState(0);

        while(!isStopRequested()) {
            follower.update();

            boolean pressed = button.isPressed();
            if (pressed && !oldPressed && stateTime.getElapsedTimeSeconds() > 0.2) {
                if (state == 0) {
//                    display.writeNumber(3);
                        display.writeCharacter('1', 0, false);
                        display.writeCharacter('6', 1, false);
                        display.writeCharacter('3', 2, false);
                        display.writeCharacter('0', 3, false);
                        display.updateDisplay(); // don't forget to call updateDisplay() or maybe do it automatically
                    changeState(10);
                } else {
                    changeState(0);
                }

            }



            boolean enter = state != oldState;
            oldState = state;

            switch (state) {
                case 0:
                    if(enter) {
                        display.writeCharacter(' ', 0, false);
                        display.writeCharacter(' ', 1, false);
                        display.writeCharacter('G', 2, false);
                        display.writeCharacter('O', 3, false);
                        display.updateDisplay();
                    }
                    follower.breakFollowing();
                    follower.setPose(startPose);
                    break;
                case 10:
                    if (enter) {
                        follower.startTeleOpDrive();
                        follower.setTeleOpDrive(1,0,0,0);
                        changeState(20);
                    }
                    break;
            }
            oldPressed = pressed;
            telemetry.addData("state", state);
            telemetry.addData("stateTime", stateTime.getElapsedTimeSeconds());
            telemetry.addData("busy", follower.isBusy());
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("buttonIsPressed", button.isPressed());
            //telemetry.addData("lift", lift.getPosition());
            //telemetry.addData("claw", claw.getPosition());
            telemetry.update();
        }
    }
//
//    private void openClaw() {
//        claw.setPosition(CLAW_OPEN);
//    }
//
//    private void liftUp() {
//        lift.setPosition(LIFT_UP);
//    }
//
//    private void liftStart() {
//        lift.setPosition(LIFT_START);
//    }
//
//    private void liftDown() {
//        lift.setPosition(LIFT_DOWN);
//    }
//    private void liftDrop() {
//        lift.setPosition(LIFT_DROP);
//    }
//
//    private void liftMedal() {
//        lift.setPosition(LIFT_MEDAL);
//    }
//
//    private void closeClaw() {
//        claw.setPosition(CLAW_CLOSED);
//    }

    private void changeState(int newState) {
        stateTime.resetTimer();
//        display.writeNumber(newState);
        display.updateDisplay();
        oldState = state;
        state = newState;
    }
}
