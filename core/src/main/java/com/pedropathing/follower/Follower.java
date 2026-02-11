package com.pedropathing.follower;

import com.pedropathing.ErrorCalculator;
import com.pedropathing.VectorCalculator;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.util.PoseHistory;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;

import java.util.ArrayDeque;
import java.util.Queue;

/**
 * This is the Follower class refactored with Black Ice power allocation strategy.
 * It prioritizes: Normal (translational) → Heading → Tangent (drive)
 * And implements reverse power clamping and improved path skipping.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 2.0, 2/7/2026 - Refactored with Black Ice strategy
 */
public class Follower {
    public FollowerConstants constants;
    public PathConstraints pathConstraints;
    public PoseTracker poseTracker;
    public ErrorCalculator errorCalculator;
    public VectorCalculator vectorCalculator;
    public Drivetrain drivetrain;
    private final PoseHistory poseHistory;
    private Pose currentPose = new Pose();
    private PathPoint closestPose = new PathPoint();
    private PathPoint previousClosestPose = new PathPoint();
    private Path currentPath = null;
    private PathChain currentPathChain = null;

    private int BEZIER_CURVE_SEARCH_LIMIT;
    private int chainIndex;
    private boolean followingPathChain, holdingPosition, isBusy, isTurning, reachedParametricPathEnd, holdPositionAtEnd, manualDrive;
    private boolean automaticHoldEnd, useHoldScaling = true;
    private double globalMaxPower = 1, centripetalScaling;
    private double holdPointTranslationalScaling;
    private double holdPointHeadingScaling;
    private double turnHeadingErrorThreshold;
    private double reversePowerClampThreshold;
    private long reachedParametricPathEndTime;
    public boolean useTranslational = true;
    public boolean useCentripetal = true;
    public boolean useHeading = true;
    public boolean useDrive = true;
    public boolean usePredictiveBraking = true;
    private Timer zeroVelocityDetectedTimer = null;
    private Runnable resetFollowing = null;
    private Queue<PathCallback> currentCallbacks;

    /**
     * This creates a new Follower given a HardwareMap.
     * @param constants FollowerConstants to use
     * @param localizer Localizer to use
     * @param drivetrain Drivetrain to use
     * @param pathConstraints PathConstraints to use
     */
    public Follower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints) {
        this.constants = constants;
        this.pathConstraints = pathConstraints;

        poseTracker = new PoseTracker(localizer);
        errorCalculator = new ErrorCalculator(constants);
        vectorCalculator = new VectorCalculator(constants);
        this.drivetrain = drivetrain;
        poseHistory = new PoseHistory(poseTracker);

        BEZIER_CURVE_SEARCH_LIMIT = constants.BEZIER_CURVE_SEARCH_LIMIT;
        holdPointTranslationalScaling = constants.holdPointTranslationalScaling;
        holdPointHeadingScaling = constants.holdPointHeadingScaling;
        centripetalScaling = constants.centripetalScaling;
        turnHeadingErrorThreshold = constants.turnHeadingErrorThreshold;
        automaticHoldEnd = constants.automaticHoldEnd;
        usePredictiveBraking = constants.usePredictiveBraking;
        reversePowerClampThreshold = 0.2; // Default clamp for reverse power

        breakFollowing();
    }

    public void updateConstants() {
        this.BEZIER_CURVE_SEARCH_LIMIT = constants.BEZIER_CURVE_SEARCH_LIMIT;
        this.holdPointTranslationalScaling = constants.holdPointTranslationalScaling;
        this.holdPointHeadingScaling = constants.holdPointHeadingScaling;
        this.centripetalScaling = constants.centripetalScaling;
        this.turnHeadingErrorThreshold = constants.turnHeadingErrorThreshold;
        this.automaticHoldEnd = constants.automaticHoldEnd;
        this.usePredictiveBraking = !manualDrive && constants.usePredictiveBraking;
    }

    /**
     * This creates a new Follower given a HardwareMap.
     * @param constants FollowerConstants to use
     * @param localizer Localizer to use
     * @param drivetrain Drivetrain to use
     */
    public Follower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain) {
        this(constants, localizer, drivetrain, PathConstraints.defaultConstraints);
    }

    public void setCentripetalScaling(double set) {
        centripetalScaling = set;
    }

    /**
     * Sets the threshold for reverse power clamping (default 0.2)
     * @param threshold the maximum power allowed when opposing velocity direction
     */
    public void setReversePowerClampThreshold(double threshold) {
        this.reversePowerClampThreshold = MathFunctions.clamp(threshold, 0, 1);
    }

    /**
     * This sets the maximum power the motors are allowed to use.
     *
     * @param set This caps the motor power from [0, 1].
     */
    public void setMaxPower(double set) {
        globalMaxPower = set;
        drivetrain.setMaxPowerScaling(set);
    }

    /**
     * This gets a Point from the current Path from a specified t-value.
     *
     * @return returns the Point.
     */
    public Pose getPointFromPath(double t) {
        if (currentPath != null) {
            return currentPath.getPoint(t);
        } else {
            return null;
        }
    }

    /**
     * This sets the current pose in the PoseTracker without using offsets.
     *
     * @param pose The pose to set the current pose to.
     */
    public void setPose(Pose pose) {
        poseTracker.setPose(pose);
    }

    /**
     * This sets the current x-position estimate of the localizer. Units are inferred from localizer constants where necessary.
     * @param x the x-position estimate to set
     */
    public void setX(double x) {
        poseTracker.getLocalizer().setX(x);
    }

    /**
     * This sets the current y-position estimate of the localizer. Units are inferred from localizer constants where necessary.
     * @param y the y-position estimate to set
     */
    public void setY(double y) {
        poseTracker.getLocalizer().setY(y);
    }

    /**
     * This sets the current heading estimate of the localizer, in radians.
     * @param heading the heading estimate to set
     */
    public void setHeading(double heading) {
        poseTracker.getLocalizer().setHeading(heading);
    }

    /**
     * This returns the current pose from the PoseTracker.
     *
     * @return returns the pose
     */
    public Pose getPose() {
        return poseTracker.getPose();
    }

    /**
     * This returns the current velocity of the robot as a Vector.
     *
     * @return returns the current velocity as a Vector.
     */
    public Vector getVelocity() {
        return poseTracker.getVelocity();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param pose the pose to set the starting pose to.
     */
    public void setStartingPose(Pose pose) {
        poseTracker.setStartingPose(pose);
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     * @param useHoldScaling true if you want to correct and turn slowly, false otherwise
     */
    public void holdPoint(BezierPoint point, double heading, boolean useHoldScaling) {
        breakFollowing();
        holdingPosition = true;
        this.useHoldScaling = useHoldScaling;
        isBusy = false;
        followingPathChain = false;
        setPath(new Path(point));
        currentPath.setConstantHeadingInterpolation(heading);
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), 1);
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    public void holdPoint(BezierPoint point, double heading) {
        holdPoint(point, heading, true);
    }

    /**
     * This holds a Point.
     *
     * @param pose the Point (as a Pose) to stay at.
     */
    public void holdPoint(Pose pose) {
        holdPoint(new BezierPoint(pose), pose.getHeading());
    }

    /**
     * This holds a Point.
     *
     * @param pose the Point (as a Pose) to stay at.
     */
    public void holdPoint(Pose pose, boolean useHoldScaling) {
        holdPoint(new BezierPoint(pose), pose.getHeading(), useHoldScaling);
    }

    /**
     * This follows a Path.
     * This also makes the Follower hold the last Point on the Path.
     *
     * @param path the Path to follow.
     * @param holdEnd this makes the Follower hold the last Point on the Path.
     */
    public void followPath(Path path, boolean holdEnd) {
        drivetrain.setMaxPowerScaling(globalMaxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = false;
        setPath(path);
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
    }

    /**
     * This follows a Path.
     *
     * @param path the Path to follow.
     */
    public void followPath(Path path) {
        followPath(path, automaticHoldEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     * @param holdEnd this makes the Follower hold the last Point on the PathChain.
     */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        followPath(pathChain, globalMaxPower, holdEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     *
     * @param pathChain the PathChain to follow.
     */
    public void followPath(PathChain pathChain) {
        followPath(pathChain, automaticHoldEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     * @param maxPower the max power of the Follower for this path
     * @param holdEnd this makes the Follower hold the last Point on the PathChain.
     */
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        drivetrain.setMaxPowerScaling(maxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        setPath(pathChain.getPath(chainIndex));
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        currentPathChain.resetCallbacks();
        currentCallbacks = currentPathChain.getNextPathCallbacks(chainIndex);

        for (PathCallback callback : currentCallbacks) {
            callback.initialize();
        }
    }

    /**
     * Resumes pathing, can only be called after pausePathFollowing()
     */
    public void resumePathFollowing() {
        if (resetFollowing != null) {
            resetFollowing.run();
            resetFollowing = null;
            breakFollowing();
            isBusy = true;
            previousClosestPose = closestPose;
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        }
    }

    /**
     * Pauses pathing, can only be restarted with resumePathFollowing
     */
    public void pausePathFollowing() {
        isBusy = false;

        boolean prevHoldEnd = holdPositionAtEnd;

        if (followingPathChain && currentPathChain != null) {
            PathChain lastChain = currentPathChain;
            int lastIndex = chainIndex;

            resetFollowing = () -> {
                followingPathChain = true;
                chainIndex = lastIndex;
                currentPathChain = lastChain;
                holdPositionAtEnd = prevHoldEnd;
                currentPath = currentPathChain.getPath(lastIndex);
            };
        } else if (currentPath != null) {
            Path lastPath = currentPath;

            resetFollowing = () -> {
                holdPositionAtEnd = prevHoldEnd;
                currentPath = lastPath;
            };
        }

        holdPoint(getPose());
    }

    /**
     * This starts teleop drive control.
     */
    public void startTeleopDrive() {
        breakFollowing();
        manualDrive = true;
        update();
        drivetrain.startTeleopDrive();
    }

    /**
     * This starts teleop drive control.
     */
    public void startTeleopDrive(boolean useBrakeMode) {
        breakFollowing();
        manualDrive = true;
        update();
        drivetrain.startTeleopDrive(useBrakeMode);
    }

    public void startTeleOpDrive(boolean useBrakeMode) {
        startTeleopDrive(useBrakeMode);
    }

    public void startTeleOpDrive() {
        startTeleopDrive();
    }

    /**
     * This sets the Teleop drive movement vectors
     *
     * @param forward the forward movement
     * @param strafe the strafe movement
     * @param turn the turn movement
     * @param isRobotCentric true if robot centric control, false if field centric
     * @param offsetHeading the offset heading for field centric control, will face the direction of such heading in radians in the field coordinate system when driving forward
     */
    public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric, double offsetHeading) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, isRobotCentric, offsetHeading);
    }

    /**
     * This sets the Teleop drive movement vectors
     *
     * @param forward the forward movement
     * @param strafe the strafe movement
     * @param turn the turn movement
     * @param offsetHeading the offset heading for field centric control, will face the direction of such heading in radians in the field coordinate system when driving forward
     */
    public void setTeleOpDrive(double forward, double strafe, double turn, double offsetHeading) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, true, offsetHeading);
    }

    /**
     * This sets the Teleop drive movement vectors
     *
     * @param forward the forward movement
     * @param strafe the strafe movement
     * @param turn the turn movement
     * @param isRobotCentric true if robot centric control, false if field centric
     */
    public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, isRobotCentric);
    }

    /**
     * This sets the Teleop drive movement vectors
     * This will default to robot centric control
     *
     * @param forward the forward movement
     * @param strafe the strafe movement
     * @param turn the turn movement
     */
    public void setTeleOpDrive(double forward, double strafe, double turn) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn);
    }

    /** Updates the Mecanum constants */
    public void updateDrivetrain() {
        drivetrain.updateConstants();
    }

    /** Calls an update to the PoseTracker, which updates the robot's current position estimate. */
    public void updatePose() {
        poseTracker.update();
        currentPose = poseTracker.getPose();
        poseHistory.update();
    }

    /** Calls an update to the ErrorCalculator, which updates the robot's current error. */
    public void updateErrors() {
        errorCalculator.update(currentPose, currentPath, currentPathChain, followingPathChain, closestPose.getPose(), poseTracker.getVelocity(), chainIndex, drivetrain.xVelocity(), drivetrain.yVelocity(), getClosestPointHeadingGoal(), usePredictiveBraking);
    }

    /** Calls an update to the VectorCalculator, which updates the robot's current vectors to correct. */
    public void updateVectors() {
        vectorCalculator.update(useDrive, useHeading, useTranslational, useCentripetal,
                manualDrive, chainIndex,
                drivetrain.getMaxPowerScaling(), followingPathChain,
                centripetalScaling, currentPose, closestPose.getPose(),
                poseTracker.getVelocity(), currentPath,
                currentPathChain, useDrive && !holdingPosition ?
                        getDriveError() : -1, getTranslationalError(),
                getHeadingError(), getClosestPointHeadingGoal(),
                getTotalDistanceRemaining(), usePredictiveBraking);
    }

    public void updateErrorAndVectors() {updateErrors(); updateVectors();}

    /**
     * Allocates power within a budget, maintaining sign
     * @param requested the requested power
     * @param budget the maximum power budget available
     * @return the allocated power
     */
    private double allocatePower(double requested, double budget) {
        return Math.copySign(
                Math.min(Math.abs(requested), budget),
                requested
        );
    }

    /**
     * Clamps reverse power to prevent fighting against velocity
     * @param power the power vector to clamp
     * @param velocity the current velocity vector
     * @return the clamped power vector
     */
    private Vector clampReversePower(Vector power, Vector velocity) {
        if (velocity.getMagnitude() < 0.01) {
            return power;
        }

        Vector velocityNormalized = velocity.normalize();
        double powerAlongVelocity = power.dot(velocityNormalized);

        if (powerAlongVelocity < 0) {
            Vector parallelComponent = velocityNormalized.times(
                    Math.max(powerAlongVelocity, -reversePowerClampThreshold)
            );
            Vector perpendicularComponent = power.minus(
                    velocityNormalized.times(powerAlongVelocity)
            );
            return parallelComponent.plus(perpendicularComponent);
        }

        return power;
    }

    /**
     * Follows a field-relative vector with heading control using Black Ice strategy
     * @param fieldVector the field-relative drive vector
     * @param headingPower the heading correction power
     */
    public void followFieldVector(Vector fieldVector, double headingPower) {
        Vector robotVector = toRobotRelativeVector(fieldVector);

        robotVector = clampReversePower(robotVector, toRobotRelativeVector(getVelocity()));

        drivetrain.followVector(robotVector, headingPower);
    }

    /**
     * Converts a field-relative vector to robot-relative
     * @param fieldVector the field-relative vector
     * @return the robot-relative vector
     */
    private Vector toRobotRelativeVector(Vector fieldVector) {
        Vector rotated = fieldVector.copy();
        rotated.rotateVector(-currentPose.getHeading());
        return rotated;
    }

    /**
     * This calls an update to the PoseTracker, which updates the robot's current position estimate.
     * This also updates all the Follower's PIDFs using Black Ice power allocation strategy.
     */
    public void update() {
        poseHistory.update();
        updateConstants();
        updatePose();
        updateDrivetrain();

        if (manualDrive) {
            previousClosestPose = closestPose;
            closestPose = new PathPoint();
            updateErrorAndVectors();
            drivetrain.runDrive(getCentripetalForceCorrection(), getTeleopHeadingVector(), getTeleopDriveVector(), poseTracker.getPose().getHeading());
            return;
        }

        if (currentPath == null) {
            return;
        }

        if (holdingPosition) {
            previousClosestPose = closestPose;
            if (followingPathChain) currentPathChain.update();
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), 1);
            updateErrorAndVectors();
            drivetrain.runDrive(useHoldScaling? getTranslationalCorrection().times(holdPointTranslationalScaling) : getTranslationalCorrection(), useHoldScaling? getHeadingVector().times(holdPointHeadingScaling) : getHeadingVector(), new Vector(), poseTracker.getPose().getHeading());

            if(Math.abs(getHeadingError()) < turnHeadingErrorThreshold && isTurning) {
                isTurning = false;
                isBusy = false;
            }
            return;
        }

        if (isBusy) {
            previousClosestPose = closestPose;
            if (followingPathChain) currentPathChain.update();
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
            updateErrorAndVectors();
            if (followingPathChain) updateCallbacks();

            Vector position = currentPose.getAsVector();
            Vector tangent = currentPath.getClosestPointTangentVector().normalize();
            Vector normal = tangent.perpendicularLeft();
            Vector velocity = getVelocity();

            // normal (translational)
            double normalError = closestPose.getPose().getAsVector().minus(position).dot(normal);
            double normalPower = vectorCalculator.getTranslationalCorrection(
                    new Vector(normalError, normal.getTheta()),
                    currentPose
            ).getMagnitude() * Math.signum(normalError);

            // tangent (drive)
            double distanceRemaining = getDistanceRemaining();
            if (closestPose.getPose().distanceFrom(currentPath.getPoint(1)) < 0.01) {
                distanceRemaining = currentPath.getPoint(1).getAsVector().minus(position).dot(tangent);
            }
            double tangentPower = vectorCalculator.getDriveVector().getMagnitude() *
                    Math.signum(distanceRemaining);

            //  heading
            double targetHeading = getClosestPointHeadingGoal();
            double headingError = getHeadingError();
            double headingPower = vectorCalculator.getHeadingVector(
                    headingError,
                    currentPose,
                    targetHeading
            ).getMagnitude() * Math.signum(headingError);

            // normal -> heading -> tangent
            double maxMagnitude = globalMaxPower;
            double normalUsed = allocatePower(normalPower, maxMagnitude);
            double remaining = Math.sqrt(
                    Math.max(0.0, maxMagnitude * maxMagnitude - normalUsed * normalUsed)
            );
            double headingUsed = allocatePower(headingPower, remaining);
            remaining = Math.sqrt(
                    Math.max(0.0, remaining * remaining - headingUsed * headingUsed)
            );
            double tangentUsed = allocatePower(tangentPower, remaining);

            Vector drivePower = normal.times(normalUsed).plus(tangent.times(tangentUsed));
            followFieldVector(drivePower, headingUsed);

            // path skipping
            if (followingPathChain && chainIndex < currentPathChain.size() - 1 &&
                    Math.abs(tangentUsed) < 1.0) {
                advanceToNextPath();
                return;
            }
        }

        if (poseTracker.getVelocity().getMagnitude() < 1.0 && currentPath.getClosestPointTValue() > 0.8
                && zeroVelocityDetectedTimer == null && isBusy) {
            zeroVelocityDetectedTimer = new Timer();
        }

        if (!(currentPath.isAtParametricEnd()
                || (zeroVelocityDetectedTimer != null
                && zeroVelocityDetectedTimer.getElapsedTime() > 500.0))) {
            return;
        }

        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
            advanceToNextPath();
            return;
        }

        if (!reachedParametricPathEnd) {
            reachedParametricPathEnd = true;
            reachedParametricPathEndTime = System.currentTimeMillis();
        }

        updateErrorAndVectors();
        if (!(
                (
                        System.currentTimeMillis() - reachedParametricPathEndTime
                                > currentPath.getPathEndTimeoutConstraint()
                )
                        || (
                        poseTracker.getVelocity().getMagnitude()
                                < currentPath.getPathEndVelocityConstraint()
                )
                        && (
                        poseTracker.getPose().distanceFrom(closestPose.getPose())
                                < currentPath.getPathEndTranslationalConstraint()
                )
                        && (
                        MathFunctions.getSmallestAngleDifference(poseTracker.getPose().getHeading(), getClosestPointHeadingGoal())
                                < currentPath.getPathEndHeadingConstraint()
                )
        )) {
            return;
        }

        if (holdPositionAtEnd) {
            holdPositionAtEnd = false;
            if (followingPathChain) holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPathChain.getHeadingGoal(new PathChain.PathT(currentPathChain.size() - 1, 1)));
            else holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPath.getHeadingGoal(1));
        } else {
            breakFollowing();
        }
    }

    /**
     * Advances to the next path in the chain and updates immediately
     */
    private void advanceToNextPath() {
        breakFollowing();
        isBusy = true;
        followingPathChain = true;
        chainIndex++;
        setPath(currentPathChain.getPath(chainIndex));
        previousClosestPose = closestPose;
        if (followingPathChain) currentPathChain.update();
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        updateErrorAndVectors();
        currentCallbacks = currentPathChain.getNextPathCallbacks(chainIndex);

        for (PathCallback callback : currentCallbacks) {
            callback.initialize();
        }
    }

    /** This checks if any PathCallbacks should be run right now, and runs them if applicable. */
    public void updateCallbacks() {
        for (PathCallback callback : currentCallbacks) {
            if (callback.isReady()) {
                callback.run();
            }
        }
    }

    /** This resets the PIDFs and stops following the current Path. */
    public void breakFollowing() {
        errorCalculator.breakFollowing();
        vectorCalculator.breakFollowing();
        drivetrain.breakFollowing();
        manualDrive = false;
        holdingPosition = false;
        isBusy = false;
        isTurning = false;
        reachedParametricPathEnd = false;
        zeroVelocityDetectedTimer = null;
    }

    public boolean isBusy() {
        return isBusy;
    }

    public PathPoint getClosestPose() {
        return closestPose;
    }

    public boolean atParametricEnd() {
        if (currentPath == null){
            return true;
        }
        if (followingPathChain) {
            if (chainIndex == currentPathChain.size() - 1) return currentPath.isAtParametricEnd();
            return false;
        }
        return currentPath.isAtParametricEnd();
    }

    public double getCurrentTValue() {
        if (isBusy) return currentPath.getClosestPointTValue();
        return 1.0;
    }

    public double getCurrentPathNumber() {
        if (!followingPathChain) return 0;
        return chainIndex;
    }

    public PathBuilder pathBuilder(PathConstraints constraints) {
        return new PathBuilder(this, constraints);
    }

    public PathBuilder pathBuilder() {
        return new PathBuilder(this);
    }

    public double getTotalHeading() {
        return poseTracker.getTotalHeading();
    }

    public Path getCurrentPath() {
        return currentPath;
    }

    public boolean isRobotStuck() {
        return zeroVelocityDetectedTimer != null;
    }

    public boolean isLocalizationNAN() {
        return poseTracker.getLocalizer().isNAN();
    }

    @Deprecated
    public void turn(double radians, boolean isLeft) {
        turn(isLeft ? radians : -radians);
    }

    public void turn(double radians) {
        Pose temp = new Pose(getPose().getX(), getPose().getY(), getPose().getHeading() + radians);
        holdPoint(temp);
        isTurning = true;
        isBusy = true;
    }

    public void turnTo(double radians) {
        holdPoint(new Pose(getPose().getX(), getPose().getY(), radians));
        isTurning = true;
        isBusy = true;
    }

    @Deprecated
    public void turnToDegrees(double degrees) {
        turnTo(Math.toRadians(degrees));
    }

    @Deprecated
    public void turnDegrees(double degrees, boolean isLeft) {
        turn(Math.toRadians(degrees), isLeft);
    }

    public boolean isTurning() {
        return isTurning;
    }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance && Math.abs(pose.getHeading() - getPose().getHeading()) < headingTolerance;
    }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance;
    }

    public void setMaxPowerScaling(double maxPowerScaling) {
        drivetrain.setMaxPowerScaling(maxPowerScaling);
    }

    public double getMaxPowerScaling() {
        return drivetrain.getMaxPowerScaling();
    }

    public boolean getUseDrive() { return useDrive; }
    public boolean getUseHeading() { return useHeading; }
    public boolean getUseTranslational() { return useTranslational; }
    public boolean getUseCentripetal() { return useCentripetal; }
    public boolean getTeleopDrive() { return manualDrive; }
    public int getChainIndex() { return chainIndex; }
    public PathChain getCurrentPathChain() { return currentPathChain; }
    public boolean getFollowingPathChain() { return followingPathChain; }
    public double getCentripetalScaling() { return centripetalScaling; }
    public boolean isTeleopDrive() { return manualDrive; }

    public Vector getTeleopHeadingVector() { return vectorCalculator.getTeleopHeadingVector(); }
    public Vector getTeleopDriveVector() { return vectorCalculator.getTeleopDriveVector(); }
    public double getHeadingError() { return errorCalculator.getHeadingError(); }
    public Vector getTranslationalError() { return errorCalculator.getTranslationalError(); }
    public double getDriveError() { return errorCalculator.getDriveError(); }
    public Vector getDriveVector() { return vectorCalculator.getDriveVector(); }
    public Vector getCorrectiveVector() { return vectorCalculator.getCorrectiveVector(); }
    public Vector getHeadingVector() { return vectorCalculator.getHeadingVector(); }
    public Vector getTranslationalCorrection() { return vectorCalculator.getTranslationalCorrection(); }
    public Vector getCentripetalForceCorrection() { return vectorCalculator.getCentripetalForceCorrection(); }
    public PathConstraints getConstraints() { return pathConstraints; }
    public FollowerConstants getConstants() { return constants; }
    public void setConstraints(PathConstraints pathConstraints) { this.pathConstraints = pathConstraints; }
    public Drivetrain getDrivetrain() { return drivetrain; }
    public PoseTracker getPoseTracker() { return poseTracker; }
    public ErrorCalculator getErrorCalculator() { return errorCalculator; }
    public VectorCalculator getVectorCalculator() { return vectorCalculator; }
    public PoseHistory getPoseHistory() { return poseHistory; }
    public void setXVelocity(double vel) { drivetrain.setXVelocity(vel); }
    public void setYVelocity(double vel) { drivetrain.setYVelocity(vel); }
    public void setDrivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) { vectorCalculator.setDrivePIDFCoefficients(drivePIDFCoefficients); }
    public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) { vectorCalculator.setSecondaryDrivePIDFCoefficients(secondaryDrivePIDFCoefficients); }
    public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) { vectorCalculator.setHeadingPIDFCoefficients(headingPIDFCoefficients); }
    public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) { vectorCalculator.setSecondaryHeadingPIDFCoefficients(secondaryHeadingPIDFCoefficients); }
    public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) { vectorCalculator.setTranslationalPIDFCoefficients(translationalPIDFCoefficients); }
    public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) { vectorCalculator.setSecondaryTranslationalPIDFCoefficients(secondaryTranslationalPIDFCoefficients); }

    public void setConstants(FollowerConstants constants) {
        this.constants = constants;
        updateConstants();
        errorCalculator.setConstants(constants);
        vectorCalculator.setConstants(constants);
        drivetrain.updateConstants();
    }

    public double getHeadingGoal(double t) {
        if (currentPathChain != null) {
            return currentPathChain.getHeadingGoal(new PathChain.PathT(chainIndex, t));
        }
        return currentPath.getHeadingGoal(t);
    }

    private double getHeadingGoal(PathPoint point) {
        if (currentPath == null) return 0;
        if (currentPathChain != null) return currentPathChain.getHeadingGoal(new PathChain.PathT(chainIndex, point.tValue));
        return currentPath.getHeadingGoal(point);
    }

    public double getClosestPointHeadingGoal() {
        if (currentPath == null) return 0;
        if (followingPathChain && currentPathChain != null)
            return currentPathChain.getClosestPointHeadingGoal(new PathChain.PathT(chainIndex, closestPose.tValue));
        return currentPath.getHeadingGoal(closestPose);
    }

    public Vector getClosestPointTangentVector() {
        return getClosestPose().getTangentVector();
    }

    public void activateAllPIDFs() {
        useDrive = true;
        useHeading = true;
        useTranslational = true;
        useCentripetal = true;
    }

    public void deactivateAllPIDFs() {
        useDrive = false;
        useHeading = false;
        useTranslational = false;
        useCentripetal = false;
    }

    public void activateDrive() { useDrive = true; }
    public void activateHeading() { useHeading = true; }
    public void activateTranslational() { useTranslational = true; }
    public void activateCentripetal() { useCentripetal = true; }

    public double getDistanceTraveledOnPath() {
        if (currentPath == null) {
            return 0;
        }
        return currentPath.getDistanceTraveled();
    }

    public double getPathCompletion() {
        if (currentPath == null) {
            return 0;
        }
        return currentPath.getPathCompletion();
    }

    public double getDistanceRemaining() {
        if (currentPath == null) {
            return 0;
        }
        return currentPath.getDistanceRemaining();
    }

    public String[] debug() {
        String[] info = new String[4];
        info[0] = poseTracker.debugString();
        info[1] = errorCalculator.debugString();
        info[2] = vectorCalculator.debugString();
        info[3] = drivetrain.debugString();
        return info;
    }

    public Vector getAcceleration() {
        return poseTracker.getAcceleration();
    }

    public double getAngularVelocity() {
        return poseTracker.getAngularVelocity();
    }

    private void setPath(Path path) {
        this.currentPath = path;
        currentPath.init();
    }

    public PathPoint getPreviousClosestPose() {
        return previousClosestPose;
    }

    public double getTangentialVelocity() {
        return getVelocity().dot(getClosestPointTangentVector().normalize());
    }

    public double getHeading() {
        return getPose().getHeading();
    }

    public double getTotalDistanceRemaining() {
        if (currentPath == null) {
            return 0;
        }
        if (!followingPathChain) {
            return currentPath.getDistanceRemaining();
        }
        PathChain.DecelerationType type = currentPathChain.getDecelerationType();
        if (type == PathChain.DecelerationType.NONE) {
            return -1;
        }
        return currentPathChain.getDistanceRemaining(chainIndex);
    }
}