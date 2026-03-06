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

import java.util.Queue;

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
    private long reachedParametricPathEndTime;
    private long lastPathAdvanceTime = 0;
    // Saved each loop inside isBusy so the skip block uses the same value — not a
    // fresh getDriveVector() call which can return inconsistent results outside isBusy.
    private double lastRawTangentPower = Double.MAX_VALUE;
    public double normalAuthority = 0.8; // Tune: lower → more braking budget, higher → tighter path tracking
    public boolean useTranslational = true;
    public boolean useCentripetal = true;
    public boolean useHeading = true;
    public boolean useDrive = true;
    public boolean usePredictiveBraking = true;
    private Timer zeroVelocityDetectedTimer = null;
    private Runnable resetFollowing = null;
    private Queue<PathCallback> currentCallbacks;

    // ── logging ──────────────────────────────────────────────────────────────
    // Throttle: only print once every LOG_INTERVAL_MS so the logcat isn't flooded.
    private static final long LOG_INTERVAL_MS = 100;
    private long lastLogTime = 0;

    private void log(String tag, String msg) {
        System.out.println("[PEDRO/" + tag + "] " + msg);
    }

    /**
     * Throttled log — fires at most once per LOG_INTERVAL_MS.
     * Use for values that change every loop (powers, errors, t-values).
     */
    private void logThrottled(String tag, String msg) {
        long now = System.currentTimeMillis();
        if (now - lastLogTime >= LOG_INTERVAL_MS) {
            lastLogTime = now;
            log(tag, msg);
        }
    }
    // ─────────────────────────────────────────────────────────────────────────

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

    public Follower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain) {
        this(constants, localizer, drivetrain, PathConstraints.defaultConstraints);
    }

    public void setCentripetalScaling(double set) { centripetalScaling = set; }

    public void setMaxPower(double set) {
        globalMaxPower = set;
        drivetrain.setMaxPowerScaling(set);
    }

    public Pose getPointFromPath(double t) {
        if (currentPath != null) return currentPath.getPoint(t);
        return null;
    }

    public void setPose(Pose pose) { poseTracker.setPose(pose); }
    public void setX(double x) { poseTracker.getLocalizer().setX(x); }
    public void setY(double y) { poseTracker.getLocalizer().setY(y); }
    public void setHeading(double heading) { poseTracker.getLocalizer().setHeading(heading); }
    public Pose getPose() { return poseTracker.getPose(); }
    public Vector getVelocity() { return poseTracker.getVelocity(); }
    public void setStartingPose(Pose pose) { poseTracker.setStartingPose(pose); }

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

    public void holdPoint(BezierPoint point, double heading) { holdPoint(point, heading, true); }
    public void holdPoint(Pose pose) { holdPoint(new BezierPoint(pose), pose.getHeading()); }
    public void holdPoint(Pose pose, boolean useHoldScaling) { holdPoint(new BezierPoint(pose), pose.getHeading(), useHoldScaling); }

    public void followPath(Path path, boolean holdEnd) {
        drivetrain.setMaxPowerScaling(globalMaxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = false;
        setPath(path);
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        log("followPath", "Started single path. holdEnd=" + holdEnd);
    }

    public void followPath(Path path) { followPath(path, automaticHoldEnd); }

    public void followPath(PathChain pathChain, boolean holdEnd) { followPath(pathChain, globalMaxPower, holdEnd); }
    public void followPath(PathChain pathChain) { followPath(pathChain, automaticHoldEnd); }

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
        for (PathCallback callback : currentCallbacks) callback.initialize();
        log("followPath", "Started PathChain. size=" + pathChain.size()
                + " maxPower=" + maxPower + " holdEnd=" + holdEnd);
    }

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

    public void startTeleopDrive() {
        breakFollowing();
        manualDrive = true;
        update();
        drivetrain.startTeleopDrive();
    }

    public void startTeleopDrive(boolean useBrakeMode) {
        breakFollowing();
        manualDrive = true;
        update();
        drivetrain.startTeleopDrive(useBrakeMode);
    }

    public void startTeleOpDrive(boolean useBrakeMode) { startTeleopDrive(useBrakeMode); }
    public void startTeleOpDrive() { startTeleopDrive(); }

    public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric, double offsetHeading) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, isRobotCentric, offsetHeading);
    }
    public void setTeleOpDrive(double forward, double strafe, double turn, double offsetHeading) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, true, offsetHeading);
    }
    public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, isRobotCentric);
    }
    public void setTeleOpDrive(double forward, double strafe, double turn) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn);
    }

    public void updateDrivetrain() { drivetrain.updateConstants(); }

    public void updatePose() {
        poseTracker.update();
        currentPose = poseTracker.getPose();
        poseHistory.update();
    }

    public void updateErrors() {
        errorCalculator.update(currentPose, currentPath, currentPathChain, followingPathChain,
                closestPose.getPose(), poseTracker.getVelocity(), chainIndex,
                drivetrain.xVelocity(), drivetrain.yVelocity(),
                getClosestPointHeadingGoal(), usePredictiveBraking);
    }

    /**
     * For intermediate paths in a NONE-deceleration PathChain, Pedro's ErrorCalculator
     * returns -1 (or a sentinel) as the drive error. VectorCalculator interprets this as
     * "max power always", keeping getDriveVector() locked at 1.0 the entire path.
     *
     * This mirrors what Black Ice's FollowPathCommand does:
     *   tangentPower = positionalController.computeOutput(distanceToEnd, vel.dot(tangent))
     * — it passes the ACTUAL remaining arc-length directly into the PD controller so
     * the output naturally drops below 1.0 as the robot approaches the waypoint.
     *
     * By substituting currentPath.getDistanceRemaining() as the drive error for
     * intermediate NONE paths, Pedro's existing drive PIDF machinery decelerates
     * naturally before each waypoint — no new tuning parameters needed.
     */
    private double getEffectiveDriveError() {
        if (!useDrive || holdingPosition) return -1;
        boolean isIntermediateNonePath = followingPathChain
                && currentPathChain != null
                && chainIndex < currentPathChain.size() - 1
                && currentPathChain.getDecelerationType() == PathChain.DecelerationType.NONE;
        if (isIntermediateNonePath) {
            return currentPath != null ? currentPath.getDistanceRemaining() : getDriveError();
        }
        return getDriveError();
    }

    public void updateVectors() {
        vectorCalculator.update(useDrive, useHeading, useTranslational, useCentripetal,
                manualDrive, chainIndex,
                drivetrain.getMaxPowerScaling(), followingPathChain,
                centripetalScaling, currentPose, closestPose.getPose(),
                poseTracker.getVelocity(), currentPath,
                currentPathChain, getEffectiveDriveError(),
                getTranslationalError(), getHeadingError(), getClosestPointHeadingGoal(),
                getTotalDistanceRemaining(), usePredictiveBraking);
    }

    public void updateErrorAndVectors() { updateErrors(); updateVectors(); }

    private double allocatePower(double requested, double budget) {
        return Math.copySign(Math.min(Math.abs(requested), budget), requested);
    }

    /**
     * Black Ice's clampReversePower: if power is opposing the direction of motion,
     * cap it to ±0.2 to prevent brownouts and allow other axes budget.
     * Applied to normalPower before allocation so the budget isn't consumed fighting momentum.
     */
    private double clampReversePower(double power, double velocity) {
        if (velocity * power >= 0) return power; // same direction or zero — no clamp
        return power < 0 ? Math.max(power, -0.2) : Math.min(power, 0.2);
    }

    public void update() {
        poseHistory.update();
        updateConstants();
        updatePose();
        updateDrivetrain();

        if (manualDrive) {
            previousClosestPose = closestPose;
            closestPose = new PathPoint();
            updateErrorAndVectors();

            double turnPower = getTeleopHeadingVector().dot(new Vector(1.0, currentPose.getHeading()));
            Vector teleopDrive = getTeleopDriveVector().copy();
            teleopDrive.rotateVector(-currentPose.getHeading());
            teleopDrive.setOrthogonalComponents(teleopDrive.getXComponent(), -teleopDrive.getYComponent());
            drivetrain.followVector(teleopDrive, turnPower, new Vector());
            return;
        }

        if (currentPath == null) return;

        if (holdingPosition) {
            previousClosestPose = closestPose;
            if (followingPathChain) currentPathChain.update();
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), 1);
            updateErrorAndVectors();
            drivetrain.runDrive(
                    useHoldScaling ? getTranslationalCorrection().times(holdPointTranslationalScaling) : getTranslationalCorrection(),
                    useHoldScaling ? getHeadingVector().times(holdPointHeadingScaling) : getHeadingVector(),
                    new Vector(),
                    poseTracker.getPose().getHeading()
            );
            if (Math.abs(getHeadingError()) < turnHeadingErrorThreshold && isTurning) {
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

            Vector tangent = currentPath.getClosestPointTangentVector().normalize();
            Vector normal  = currentPath.getClosestLeftGradientVector().normalize();

            Vector fieldVelocity = poseTracker.getVelocity();
            double velocityDotNormal  = fieldVelocity.dot(normal);
            double velocityDotTangent = fieldVelocity.dot(tangent);

            double normalPower = getCorrectiveVector().dot(normal) * normalAuthority;

            double rawTangentPower = getDriveVector().dot(tangent);
            lastRawTangentPower = rawTangentPower;
            boolean isIntermediatePath = followingPathChain && chainIndex < currentPathChain.size() - 1;

            double tangentPower = rawTangentPower;
            double headingPower = getHeadingVector().dot(new Vector(1.0, currentPose.getHeading()));

            // NORMAL, HEADING, TANGENT

            double normalUsed  = allocatePower(normalPower, globalMaxPower);
            double rem1 = Math.sqrt(Math.max(0.0, globalMaxPower * globalMaxPower - normalUsed * normalUsed));
            double headingUsed = allocatePower(headingPower, rem1);
            double rem2 = Math.sqrt(Math.max(0.0, rem1 * rem1 - headingUsed * headingUsed));
            double tangentUsed = allocatePower(tangentPower, rem2);

            if (reachedParametricPathEnd || currentPath.getClosestPointTValue() > 0.98) {
                tangentUsed = 0.0;
            }

            Vector fieldDrivePower = normal.times(normalUsed).plus(tangent.times(tangentUsed));

            Vector robotDrivePower = fieldDrivePower.copy();
            robotDrivePower.rotateVector(-currentPose.getHeading());
            robotDrivePower.setOrthogonalComponents(
                    robotDrivePower.getXComponent(),
                    -robotDrivePower.getYComponent()
            );

            Vector robotVelocity = poseTracker.getVelocity().copy();
            robotVelocity.rotateVector(-currentPose.getHeading());
            robotVelocity.setOrthogonalComponents(
                    robotVelocity.getXComponent(),
                    -robotVelocity.getYComponent()
            );

            // ── LOG A: main drive values (throttled) ──────────────────────────
            // Shows the raw computed powers before and after allocation.
            // If normalPower is huge but normalUsed is small → budget is starved by heading/tangent.
            // If tangentPower > 0 but tangentUsed = 0 → deadband or reachedParametricPathEnd zeroing it.
            // robotDrivePower X = forward/back in robot frame, Y = lateral.
            logThrottled("DRIVE",
                    "pos=[" + String.format("%.2f", currentPose.getX())
                            + "," + String.format("%.2f", currentPose.getY()) + "]"
                            + " t=" + String.format("%.3f", currentPath.getClosestPointTValue())
                            + " distRem=" + String.format("%.1f", currentPath.getDistanceRemaining())
                            + " effDriveErr=" + String.format("%.1f", getEffectiveDriveError())
                            + " normPwr=" + String.format("%.3f", normalPower)
                            + " normUsed=" + String.format("%.3f", normalUsed)
                            + " vDotN=" + String.format("%.2f", velocityDotNormal)
                            + " rawTan=" + String.format("%.3f", rawTangentPower)
                            + " tanUsed=" + String.format("%.3f", tangentUsed)
                            + " vDotT=" + String.format("%.2f", velocityDotTangent)
                            + " hdgUsed=" + String.format("%.3f", headingUsed)
                            + " rem1=" + String.format("%.3f", rem1)
                            + " rem2=" + String.format("%.3f", rem2)
                            + " drive=[" + String.format("%.3f", robotDrivePower.getXComponent())
                            + "," + String.format("%.3f", robotDrivePower.getYComponent()) + "]"
                            + " isInter=" + isIntermediatePath
            );

            drivetrain.followVector(robotDrivePower, headingUsed, robotVelocity);
        }

        // ── LOG B: stuck-detection trigger ────────────────────────────────────
        // Fires once when the timer starts. If you see this and then the robot still
        // doesn't stop, the timeout (500 ms) is either expiring and doing the right
        // thing, or the timer isn't being checked below.
        if (poseTracker.getVelocity().getMagnitude() < 1.0 && currentPath.getClosestPointTValue() > 0.8
                && zeroVelocityDetectedTimer == null && isBusy) {
            zeroVelocityDetectedTimer = new Timer();
            log("STUCK", "Zero-velocity timer started. t="
                    + String.format("%.3f", currentPath.getClosestPointTValue())
                    + " vel=" + String.format("%.3f", poseTracker.getVelocity().getMagnitude()));
        }

        // ── LOG C: path-skip inputs (throttled) ───────────────────────────────
        // lastRawTangentPower: drive PIDF output projected on path tangent (correct value).
        // When Pedro's predictive braking works this drops below globalMaxPower ~0.5-1s
        // before path end. If it stays at exactly globalMaxPower → braking disabled for
        // this path (DecelerationType.NONE), advance will happen at isAtParametricEnd.
        logThrottled("SKIP",
                "chainIdx=" + chainIndex
                        + " t=" + String.format("%.3f", currentPath.getClosestPointTValue())
                        + " rawTan=" + String.format("%.3f", lastRawTangentPower)
                        + " maxPwr=" + String.format("%.3f", globalMaxPower)
                        + " isAtEnd=" + currentPath.isAtParametricEnd()
                        + " willSkip="
                        + (followingPathChain && chainIndex < currentPathChain.size() - 1
                        && usePredictiveBraking
                        && currentPath.getClosestPointTValue() > 0.5
                        && lastRawTangentPower < globalMaxPower)
                        + " stuckTimer=" + (zeroVelocityDetectedTimer != null
                        ? String.format("%.0f ms", zeroVelocityDetectedTimer.getElapsedTime())
                        : "null")
        );

        boolean nextPathWithinBrakingDistance =
                followingPathChain
                        && chainIndex < currentPathChain.size() - 1
                        && usePredictiveBraking
                        && currentPath.getClosestPointTValue() > 0.5
                        && lastRawTangentPower < globalMaxPower;

        if (!(currentPath.isAtParametricEnd()
                || nextPathWithinBrakingDistance
                || (zeroVelocityDetectedTimer != null
                && zeroVelocityDetectedTimer.getElapsedTime() > 500.0))) {
            return;
        }

        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
            // ── LOG D: path advance ───────────────────────────────────────────
            // Fires exactly once per path transition.
            // If you see this firing before the robot has visibly reached the path end
            // → the skip condition is triggering too early (check LOG C tangentDot values).
            log("ADVANCE", "Advancing from path " + chainIndex + " → " + (chainIndex + 1)
                    + " reason: isAtEnd=" + currentPath.isAtParametricEnd()
                    + " nextWithin=" + nextPathWithinBrakingDistance
                    + " stuck=" + (zeroVelocityDetectedTimer != null
                    && zeroVelocityDetectedTimer.getElapsedTime() > 500.0)
                    + " t=" + String.format("%.3f", currentPath.getClosestPointTValue())
                    + " rawTan=" + String.format("%.3f", lastRawTangentPower));
            advanceToNextPath();
            return;
        }

        if (!reachedParametricPathEnd) {
            long timeSinceAdvance = System.currentTimeMillis() - lastPathAdvanceTime;
            if (timeSinceAdvance < 100) {
                log("END-GUARD", "Skipping reachedParametricPathEnd set — too soon after advance ("
                        + timeSinceAdvance + " ms)");
                return;
            }
            reachedParametricPathEnd = true;
            reachedParametricPathEndTime = System.currentTimeMillis();
            // ── LOG E: parametric end reached ────────────────────────────────
            // Fires once when the robot is considered "at the end" of the last path.
            // After this, tangentUsed is forced to 0 (see LOG A).
            // If the robot is still moving sideways after this → normal correction is
            // the culprit (normalPower non-zero and perpendicular to intended direction).
            log("END", "reachedParametricPathEnd=true"
                    + " pos=[" + String.format("%.2f", currentPose.getX())
                    + "," + String.format("%.2f", currentPose.getY()) + "]"
                    + " vel=" + String.format("%.3f", poseTracker.getVelocity().getMagnitude())
                    + " t=" + String.format("%.3f", currentPath.getClosestPointTValue()));
        }

        updateErrorAndVectors();

        double velMag    = poseTracker.getVelocity().getMagnitude();
        double transDist = poseTracker.getPose().distanceFrom(closestPose.getPose());
        double headingErr = MathFunctions.getSmallestAngleDifference(
                poseTracker.getPose().getHeading(), getClosestPointHeadingGoal());
        long   msAtEnd   = System.currentTimeMillis() - reachedParametricPathEndTime;

        // ── LOG F: end-constraint check (throttled) ───────────────────────────
        // Shows every value checked to decide when the path is "done".
        // velMag must be < getPathEndVelocityConstraint()
        // transDist must be < getPathEndTranslationalConstraint()
        // headingErr must be < getPathEndHeadingConstraint()
        // OR msAtEnd > getPathEndTimeoutConstraint() to force-finish.
        // If the robot is drifting and these never pass → normal correction is
        // adding lateral velocity faster than the constraints allow it to expire.
        logThrottled("END-CHECK",
                "msAtEnd=" + msAtEnd
                        + " timeout=" + currentPath.getPathEndTimeoutConstraint()
                        + " velMag=" + String.format("%.3f", velMag)
                        + " velLimit=" + String.format("%.3f", currentPath.getPathEndVelocityConstraint())
                        + " transDist=" + String.format("%.3f", transDist)
                        + " transLimit=" + String.format("%.3f", currentPath.getPathEndTranslationalConstraint())
                        + " headErr=" + String.format("%.4f", headingErr)
                        + " headLimit=" + String.format("%.4f", currentPath.getPathEndHeadingConstraint())
        );

        boolean timeoutDone  = msAtEnd > currentPath.getPathEndTimeoutConstraint();
        boolean constraintsDone = velMag  < currentPath.getPathEndVelocityConstraint()
                && transDist < currentPath.getPathEndTranslationalConstraint()
                && headingErr < currentPath.getPathEndHeadingConstraint();

        if (!timeoutDone && !constraintsDone) return;

        // ── LOG G: path complete ──────────────────────────────────────────────
        // Fires once when the path is declared done.
        // "timeout" means it finished because time ran out (constraints weren't met).
        // "constraints" means it settled cleanly.
        log("DONE", "Path complete. reason=" + (timeoutDone ? "timeout" : "constraints")
                + " holdPositionAtEnd=" + holdPositionAtEnd);

        if (holdPositionAtEnd) {
            holdPositionAtEnd = false;
            if (followingPathChain)
                holdPoint(new BezierPoint(currentPath.getLastControlPoint()),
                        currentPathChain.getHeadingGoal(new PathChain.PathT(currentPathChain.size() - 1, 1)));
            else
                holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPath.getHeadingGoal(1));
        } else {
            breakFollowing();
        }
    }

    private void advanceToNextPath() {
        breakFollowing();
        isBusy = true;
        followingPathChain = true;
        chainIndex++;
        lastPathAdvanceTime = System.currentTimeMillis();
        setPath(currentPathChain.getPath(chainIndex));
        previousClosestPose = closestPose;
        if (followingPathChain) currentPathChain.update();
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        updateErrorAndVectors();
        currentCallbacks = currentPathChain.getNextPathCallbacks(chainIndex);
        for (PathCallback callback : currentCallbacks) callback.initialize();
    }

    public void updateCallbacks() {
        for (PathCallback callback : currentCallbacks) {
            if (callback.isReady()) callback.run();
        }
    }

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
        lastRawTangentPower = Double.MAX_VALUE; // reset so stale value never triggers skip on new path
    }

    public boolean isBusy() { return isBusy; }
    public PathPoint getClosestPose() { return closestPose; }

    public boolean atParametricEnd() {
        if (currentPath == null) return true;
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

    public PathBuilder pathBuilder(PathConstraints constraints) { return new PathBuilder(this, constraints); }
    public PathBuilder pathBuilder() { return new PathBuilder(this); }
    public double getTotalHeading() { return poseTracker.getTotalHeading(); }
    public Path getCurrentPath() { return currentPath; }
    public boolean isRobotStuck() { return zeroVelocityDetectedTimer != null; }
    public boolean isLocalizationNAN() { return poseTracker.getLocalizer().isNAN(); }

    @Deprecated
    public void turn(double radians, boolean isLeft) { turn(isLeft ? radians : -radians); }

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
    public void turnToDegrees(double degrees) { turnTo(Math.toRadians(degrees)); }
    @Deprecated
    public void turnDegrees(double degrees, boolean isLeft) { turn(Math.toRadians(degrees), isLeft); }

    public boolean isTurning() { return isTurning; }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance
                && Math.abs(pose.getY() - getPose().getY()) < yTolerance
                && Math.abs(pose.getHeading() - getPose().getHeading()) < headingTolerance;
    }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance
                && Math.abs(pose.getY() - getPose().getY()) < yTolerance;
    }

    public void setMaxPowerScaling(double maxPowerScaling) { drivetrain.setMaxPowerScaling(maxPowerScaling); }
    public double getMaxPowerScaling() { return drivetrain.getMaxPowerScaling(); }
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
        if (currentPathChain != null)
            return currentPathChain.getHeadingGoal(new PathChain.PathT(chainIndex, t));
        return currentPath.getHeadingGoal(t);
    }

    private double getHeadingGoal(PathPoint point) {
        if (currentPath == null) return 0;
        if (currentPathChain != null)
            return currentPathChain.getHeadingGoal(new PathChain.PathT(chainIndex, point.tValue));
        return currentPath.getHeadingGoal(point);
    }

    public double getClosestPointHeadingGoal() {
        if (currentPath == null) return 0;
        if (followingPathChain && currentPathChain != null)
            return currentPathChain.getClosestPointHeadingGoal(new PathChain.PathT(chainIndex, closestPose.tValue));
        return currentPath.getHeadingGoal(closestPose);
    }

    public Vector getClosestPointTangentVector() { return getClosestPose().getTangentVector(); }

    public void activateAllPIDFs() { useDrive = true; useHeading = true; useTranslational = true; useCentripetal = true; }
    public void deactivateAllPIDFs() { useDrive = false; useHeading = false; useTranslational = false; useCentripetal = false; }
    public void activateDrive() { useDrive = true; }
    public void activateHeading() { useHeading = true; }
    public void activateTranslational() { useTranslational = true; }
    public void activateCentripetal() { useCentripetal = true; }

    public double getDistanceTraveledOnPath() {
        if (currentPath == null) return 0;
        return currentPath.getDistanceTraveled();
    }

    public double getPathCompletion() {
        if (currentPath == null) return 0;
        return currentPath.getPathCompletion();
    }

    public double getDistanceRemaining() {
        if (currentPath == null) return 0;
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

    public Vector getAcceleration() { return poseTracker.getAcceleration(); }
    public double getAngularVelocity() { return poseTracker.getAngularVelocity(); }
    private void setPath(Path path) { this.currentPath = path; currentPath.init(); }
    public PathPoint getPreviousClosestPose() { return previousClosestPose; }
    public double getTangentialVelocity() { return getVelocity().dot(getClosestPointTangentVector().normalize()); }
    public double getHeading() { return getPose().getHeading(); }

    public double getTotalDistanceRemaining() {
        if (currentPath == null) return 0;
        if (!followingPathChain) return currentPath.getDistanceRemaining();
        PathChain.DecelerationType type = currentPathChain.getDecelerationType();

        if (type == PathChain.DecelerationType.NONE) return currentPath.getDistanceRemaining();
        return currentPathChain.getDistanceRemaining(chainIndex);
    }
}