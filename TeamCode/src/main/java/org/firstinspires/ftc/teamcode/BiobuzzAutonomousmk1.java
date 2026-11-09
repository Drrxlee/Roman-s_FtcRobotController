package org.firstinspires.ftc.teamcode;




import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teamPedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamPedroPathing.TeamPoses;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.HeadingInterpolationType;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;

/* config to make work
* pipeline 1 = blue Alliance, so add filters and processes to cameras
* and frames specifically for blue alliance AprilTags
*
* pipeline 2 = red Alliance, so add filters and processes to cameras
 * and frames specifically for red alliance  */
@Autonomous
public class BiobuzzAutonomousmk1 extends OpMode {

    // private StartingSide startingSide = StartingSide.HIVEUP;
    private AllianceColor allianceColor = AllianceColor.RED;
    private DcMotorEx testshooter = null;
    private double currentPower = 0;

    private RobotHardware robotHardware;
    private PedroPather pedroPather;
    private PedroMotion pedroMotion;
    private Follower follower;
    private Limelight3A limelight;
    private int currentAction = 0;

    private boolean stateInitialized = false;
    /*
    The code above is waking up the limelight and flags it to make sure
    it is paths & timers only "init" once per state
    */
    private Timer shooterTimer;
    private Timer parkTimer;
    private final double TICKS_PER_REV = 28.0;

    @Override
    public void init() {
        robotHardware = new RobotHardware(this);
        follower = Constants.create(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.start();

        pedroMotion = new PedroMotion(follower);
        parkTimer = new Timer();
        shooterTimer = new Timer();
    }
    /* what does it do? basically adds a live steam to the port 5801, so coding can see if
    its actively working
    *
    * also sends this info continuously to the driver hub to know which one is currently selected */
    @Override
    public void init_loop() {
        // this is telling the limelight what color team you are on
        allianceColor = AllianceColor.INSTANCE.toggleColor(gamepad1.dpadUpWasPressed(), allianceColor);
        allianceColor = AllianceColor.INSTANCE.toggleColor(gamepad1.dpadDownWasPressed(), allianceColor);

        // switches the active pipeline (edited per alliance color)
        int activePipeline = (allianceColor == AllianceColor.BLUE) ? 1 : 2;
        limelight.pipelineSwitch(activePipeline);

        testshooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        testshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        testshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // what pops up on the screen.
        telemetry.addData("=== Init Menu ===", "");
        telemetry.addData("Controls", "D-Pad UP / DOWN to switch alliance");
        telemetry.addData("Selected Alliance", allianceColor.toString());
        telemetry.addData("Active Pipeline", "%d (%s Target)", activePipeline, allianceColor.toString());
        telemetry.update();
    }

    public void start() {
        // locks in the final pipeline selection when you press start.
        int finalPipeline = (allianceColor == AllianceColor.BLUE) ? 1 : 2;
        limelight.pipelineSwitch(finalPipeline);

        pedroPather = new PedroPather(TeamPoses.canonicalColor, allianceColor, PedroPather::FlipRoutineAlliancesOnOppositeSides);
        parkTimer.reset();

        // Reset state flag so case can init properly
        stateInitialized = false;

    }
/* what's the limelight is currently doing
* Limelight is scanning every frame for 3d aprilTag fiducial markers
* limelight is also scanning the horizontal offset and vertical offset
* along with other 3D space vectors (distance and stuff)
*
* case 6 - checks if valid target data is being given
* target lock confirmation to driver station logs
* provides the vision verification needed while the shooter runs for 3 seconds
* "shootUntilComplete()"
* before handing the controls back to pedroPathing for the final park state  */
    @Override
    public void loop() {
        follower.update();
        LLResult llResult = limelight.getLatestResult();

        testshooter.setPower(currentPower);
        double currentTicksPerSec = testshooter.getVelocity();
        double currentRpm = (currentTicksPerSec / TICKS_PER_REV) * 60.0;

        telemetry.clear();

        telemetry.addData("this is the currently selected alliance", allianceColor.toString());
        telemetry.addData("this is the side Currently selected", allianceColor.toString()); // placeholder
        telemetry.addData("----------------------", "");
        telemetry.addData("Set Power", "%.2f", currentPower);
        telemetry.addData("Target Velocity (ticks/s)", currentTicksPerSec);
        telemetry.addData("Calculated Speed (RPM)", "%.1f RPM", currentRpm);
        /* if(timeToPark()){
            pedroMotion.goPath(pedroPather.pathBetween(follower.pose(), TeamPoses.parkPose, HeadingInterpolationType.LINEAR));
            if(pedroMotion.isPathComplete()){
                requestOpModeStop();
            }
         */
        if (timeToPark() && currentAction < 7) {
            setNextState(7); // Jump straight to park state
        }
        switch (currentAction) {
            case 0: // Initial Shot
                // Resets shooterTimer once it is entering Case 0
                if (!stateInitialized) {
                    shooterTimer.reset();
                    stateInitialized = true;
                }
                if (shootUntilComplete()) {
                    setNextState(1); //Replaced direct variable update with setNextState()
                }
                break;

            case 1: // Turns Intake On
                robotHardware.intakeOn();
                setNextState(2);
                break;

            case 2: // Drives to the Garden
                // Path generated ONCE on state start instead of every loop
                if (!stateInitialized) {
                    pedroMotion.goPath(pedroPather.pathBetween(TeamPoses.startShootPose, TeamPoses.startGardenPose, HeadingInterpolationType.LINEAR));
                    stateInitialized = true;
                }
                if (pedroMotion.isPathComplete()) {
                    setNextState(3);
                }
                break;

            case 3: // Intake Sweep Through Garden (50% Speed)
                //Path generated once on state start
                if (!stateInitialized) {
                    pedroMotion.goPath(pedroPather.pathBetween(TeamPoses.startGardenPose, TeamPoses.endGardenPose, HeadingInterpolationType.LINEAR), 0.5);
                    stateInitialized = true;
                }
                if (pedroMotion.isPathComplete()) {
                    setNextState(4);
                }
                break;

            case 4: // Turns the intake Off
                robotHardware.intakeOff();
                setNextState(5);
                break;

            case 5: // Drives back to Shoot Pose
                //Path generated once on state start
                if (!stateInitialized) {
                    pedroMotion.goPath(pedroPather.pathBetween(TeamPoses.startGardenPose, TeamPoses.startShootPose, HeadingInterpolationType.LINEAR));
                    stateInitialized = true;
                }
                if (pedroMotion.isPathComplete()) {
                    setNextState(6);
                }
                break;

            case 6: // Limelight Target Verification & Second Shot
                // Resets shooterTimer ONCE when entering Case 6
                if (!stateInitialized) {
                    shooterTimer.reset();
                    stateInitialized = true;
                }

                // Improved target presence check to prevent vision lock freezes
                boolean hasTarget = llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty();
                telemetry.addData("Limelight Target Found", hasTarget);

                if (shootUntilComplete()) {
                    setNextState(7);
                }
                break;

            case 7: // Park
                // Path generated Once on state start
                if (!stateInitialized) {
                    pedroMotion.goPath(pedroPather.pathBetween(follower.pose(), TeamPoses.parkPose, HeadingInterpolationType.LINEAR));
                    stateInitialized = true;
                }
                if (pedroMotion.isPathComplete()) {
                    requestOpModeStop();
                }
                break;
        }
    }

    //helper method added to cleanly handle state transfer
    private void setNextState(int nextState) {
        currentAction = nextState;
        stateInitialized = false;
    }
    private boolean shootUntilComplete() {
        boolean isComplete;
        robotHardware.shooterON();

        isComplete = shooterTimer.milliseconds() > 3000;
        if(isComplete){
            robotHardware.shooterOFF();
        }
        return isComplete;

    }
    private boolean timeToPark(){
        return parkTimer.seconds() > 15;

    }
}