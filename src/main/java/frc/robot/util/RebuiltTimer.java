package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Timer class to keep track of hub state + additional information in 2026 game Rebuilt
 * @author Anvay Mathur
 */
public class RebuiltTimer {

    public static enum HubState {
        RED,
        BLUE,
        BOTH
    }

    public static enum MatchState{
        AUTO,
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME
    }

    public static enum AutoWinner {
        RED,
        BLUE,
        ERROR
    }

    public static enum GameState{
        ACQUIRE,
        LAUNCH,
        PASS,
        IDLE,
        TRAVEL
    }

    private HubState activeHub;
    public MatchState currentShift;
    private boolean redIsActiveFirst;
    public AutoWinner autoWinner;
    private Timer timerFMS;
    private long timer;
    private int fuel;
    public double timeInShift;

    private HashMap<GameState, Double> gameStrat; 
    private GameState[] gameStrategy;

//  TODO: Get accurate values
    private final double FUEL_PER_SECOND = 7;
    private final LinearVelocity ROBOT_SPEED = MetersPerSecond.of(4);




    public RebuiltTimer() {
        this.activeHub = HubState.BOTH;
        this.currentShift = MatchState.AUTO;
        this.redIsActiveFirst = false;
        this.autoWinner = AutoWinner.ERROR;
        this.timerFMS = new Timer();
        this.timer = -1;
        this.fuel = 0;
        this.gameStrat = new HashMap<>();
    }

    /**
     * Start timer (not needed if using {@link DriverStation#getMatchTime()})
     */
    public void start(){
        timer = System.currentTimeMillis();
        timerFMS.start();
    }

    /**
     * Gets the game specific message about who won auto from {@link DriverStation#getGameSpecificMessage()}
     * and returns the alliance that won auto. If no message is recieved or message is corrupted, returns error state.
     * @return the alliance that won auto, or an error state {@link AutoWinner}
     */
    public AutoWinner getAutoWinner(){

        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.length()>0){
            switch (gameData.charAt(0)){
                case 'B'-> {
                    autoWinner = AutoWinner.BLUE;
                    redIsActiveFirst = false;
                } case 'R'->{
                    autoWinner = AutoWinner.RED;
                    redIsActiveFirst = true;
                } default -> {
                    autoWinner = AutoWinner.ERROR;
                }
            }
        }

        // TODO: Have an elastic notification for manual ovveride from operator
        if ((DriverStation.isTeleop() && gameData.isEmpty()) || autoWinner == AutoWinner.ERROR){
            Elastic.Notification autoErrorNotif = 
                new Elastic.Notification(
                    Elastic.NotificationLevel.ERROR, 
                    "AUTO Winner Not Detected", 
                    "The AUTO winner was not detected, please manually input the first active hub"
                );            
                autoWinner = AutoWinner.ERROR;
                SmartDashboard.getBoolean("Red is active first?", redIsActiveFirst);
        }

        return this.autoWinner;
    }
//    get match time
//          Manual match Time or Timer class, or Fms match time
    /**
     * Counts down the match time from 160 seconds
     * If using {@link DriverStation#getMatchTime()} auto counts down from 20 and teleop from 140
     * @return Match time
     */
    public double getMatchTime(){
        // return Timer.getMatchTime();
        return DriverStation.getMatchTime();
        // return 160 - timerFMS.get();
        // return 160 - (System.currentTimeMillis() - timer);
    }

    // Tracks when the shifts change
    /**
     * Tracks the shift state using the match time from {@link #getMatchTime()} 
     */
    public void trackShift(){
        double matchTime = DriverStation.getMatchTime();
        // Transition Time
        if (DriverStation.isAutonomous()){
            // Auto
            currentShift = MatchState.AUTO;

        } else if (matchTime > 130){
            // Transition
            activeHub = HubState.BOTH;
            currentShift = MatchState.TRANSITION;

        } else if (matchTime > 105){
            // Shift 1
            if (redIsActiveFirst)
                activeHub = HubState.RED;
            else 
                activeHub = HubState.BLUE;

            currentShift = MatchState.SHIFT1;

        } else if (matchTime > 80){
            // Shift 2
            if (redIsActiveFirst)
                activeHub = HubState.BLUE;
            else
                activeHub = HubState.RED;

            currentShift = MatchState.SHIFT2;
        } else if(matchTime > 55){
            // Shift 3
            if (redIsActiveFirst)
                activeHub = HubState.RED;
            else 
                activeHub = HubState.BLUE;

            currentShift = MatchState.SHIFT3;

        } else if (matchTime > 30){
            // Shift 4
            if (redIsActiveFirst)
                activeHub = HubState.BLUE;
            else
                activeHub = HubState.RED;
            currentShift = MatchState.SHIFT4;

        } else {
            // Endgame
            activeHub = HubState.BOTH;
            currentShift = MatchState.ENDGAME;
        }
    }



//    Get time to current shift
//          Track shift time
// Track how much time is left in the current shift in seconds. counts down from shift start to shift end
    /**
     * Tracks how much time is left in the current shift, counting down from shift start time to shift end time.
     * @return time left in current shift
     */
    public double getTimeInShift(){
        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous())
            return matchTime;
        else if (currentShift == MatchState.TRANSITION)
            timeInShift = matchTime - 130;
        else if (currentShift == MatchState.SHIFT1)
            timeInShift = matchTime - 105;
        else if (currentShift == MatchState.SHIFT2)
            timeInShift = matchTime - 80;
        else if (currentShift == MatchState.SHIFT3)
            timeInShift = matchTime - 55;
        else if (currentShift == MatchState.SHIFT4)
            timeInShift = matchTime - 30;
        else if(currentShift == MatchState.ENDGAME)
            timeInShift = matchTime;
        else 
            return 0.0;
        
        return timeInShift;
    }

//    Get time to next shift
//         Track shift time

//    Active? Inactive?
//          Hub state tracking, track shift time
    /**
     * Checks to see if the active hub is the same as the current alliance
     * @return if the hub is active
     */
    public boolean isHubActive(){
        if (activeHub == HubState.BOTH)
            return true;
        if (DriverStation.getAlliance().get() == Alliance.Red && activeHub == HubState.RED)
            return true;
        else if (DriverStation.getAlliance().get() == Alliance.Blue && activeHub == HubState.BLUE)
            return true;
        
        return false;
    }

//    Get time to get back to 
//          Calculate distance relative to time
    /**
     * time it takes to get from robot pose to target pose, each axis is added individually.
     * So it imagines you only move in straight lines horizontally and vertically. 
     * Uses {@link #ROBOT_SPEED} to calculate the time
     * @param robot robot pose
     * @param target target pose
     * @return time it takes to get from robot pose to target pose
     */
    public Time timeToPose(Supplier<Pose2d> robot, Supplier<Pose2d> target){
        Pose2d robotPose = robot.get();
        Pose2d targetPose = target.get();

        Distance xDiff = targetPose.getMeasureX().minus(robotPose.getMeasureX());
        Distance yDiff = targetPose.getMeasureY().minus(robotPose.getMeasureY());

        xDiff = Meters.of(xDiff.abs(Meters));
        yDiff = Meters.of(yDiff.abs(Meters));

        Time xTime = xDiff.div(ROBOT_SPEED);
        Time yTime = yDiff.div(ROBOT_SPEED);


        return xTime.plus(yTime);
    }

    public Pose2d getClosestScoringPosition(Supplier<Pose2d> robot){
        Pose2d robotPose = robot.get();
        
        return Pose2d.kZero;
    }

//    Time left to acquire
//         Time left in hub  - Time to shoot - time to get back 
    /**
     * The amount of time left to be acquire in the shift based on how long it takes to score and how long it takes to get to scoring position
     * @param scoreTime how long it takes to score
     * @param timeToPose how long it takes to get to scoring position
     * @return the time left in the shift availaible for acquiring
     */
    public double timeToAcquire(double scoreTime, Time timeToPose){
        return timeInShift - scoreTime - timeToPose.in(Seconds);
    }
    
//    Next "Phase"

//    Next Action

//    Time needed/left to shoot
//         Fuel in robot / (fuel/second)
    /**
     * Time needed to score based on the amount of {@link #fuel} and the set {@link #FUEL_PER_SECOND}
     * @return time needed to score
     */
    public double scoreTime(){
        return fuel / FUEL_PER_SECOND;
    }

//    Fuel Guage
    /**
     * Set the amount of fuel the robot has {@link #fuel}
     * @param fuel the amount of fuel to set
     */
    public void setFuel(int fuel){
        this.fuel = fuel;
    }

    /**
     * Add to the amount of fuel the robot has {@link #fuel}
     * @param fuel the amount of fuel to add
     */
    public void addFuel(int fuel){
        this.fuel += fuel;
    }

    /**
     * Gets the amount of {@link #fuel} in the robot
     * @return the amount of fuel
     */
    public int getFuel(){
        return this.fuel;
    }

//    Time left to Feed
//      time in shift - (time needed to get back + time needed to shoot) and something else

//    Button: Current Action Done

//    Idle time

//    Map out match stratergy
    /**
     * Set the match strategy as a hashmap of {@link GameState} and time
     * @param gameStrat hashmap of {@link GameState} and time
     */
    public void setGameStrategy(HashMap<GameState,Double> gameStrat){
        this.gameStrat = gameStrat;
    }

    /**
     * Gets the match strategy as a hashmap of {@link GameState} and time (double)
     * @return
     */
    public HashMap<GameState,Double> getGameStrat(){
        return gameStrat;
    }

    /**
     * Sets the game stratery as an array of {@link GameState} in order of events
     * @param gameStrat the game strategy
     */
    public void setGameStrategy(GameState[] gameStrat){
        this.gameStrategy = gameStrat;
    }

    /**
     * Gets the current game strategy as an array of {@link GameState} in order of events
     * @return
     */
    public GameState[] getGameStrategy(){
        return gameStrategy;
    }   
    

    
}
