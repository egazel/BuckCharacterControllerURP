using UnityEngine;
using KinematicCharacterController;
using UnityEngine.Windows;

public enum CrouchInput
{
    None, Toggle
}

public enum GrappleInput
{
    None, Toggle
}

public enum Stance
{
    Stand, Crouch, Slide, Dash, Grapple, WallRun
}

public struct CharacterState
{
    public bool Grounded;
    public Stance Stance;
    public Vector3 Velocity;
    public Vector3 Acceleration;
}

public struct CharacterInput
{
    public Quaternion Rotation;
    public Vector2 Move;
    public bool Jump;
    public bool Dash;
    public bool JumpSustain;
    public CrouchInput Crouch;
    public GrappleInput Grapple;
}

public class PlayerCharacter : MonoBehaviour, ICharacterController
{
    [SerializeField] private KinematicCharacterMotor motor;
    [SerializeField] private Transform root;
    [SerializeField] private Transform cameraTarget;
    [SerializeField] private Transform cameraTransform;
    [Space]
    [Header("Walk/Crouch speed")]
    [SerializeField] private float walkSpeed = 20f;
    [SerializeField] private float crouchSpeed = 7f;
    [SerializeField] private float walkResponse = 25f;
    [SerializeField] private float crouchResponse = 20f;
    [SerializeField] float maxPlanarSpeed = 50f;
    [SerializeField] float maxVerticalSpeed = 50f;
    [Space]
    [Header("Air control")]
    [SerializeField] private float airSpeed = 15f;
    [SerializeField] private float airAcceleration = 70f;
    [Space]
    [Header("Jump")]
    [SerializeField] private float jumpSpeed = 20f;
    [SerializeField] private float coyoteTime = 0.2f;
    [SerializeField] private float numberOfJumps = 2f;
    [Range(0f, 1f)]
    [SerializeField] private float jumpSustainGravity = 0.7f;
    [SerializeField] private float gravity = -90f;
    [Space]
    [Header("Slide")]
    [SerializeField] private float slideStartSpeed = 30f;
    [SerializeField] private float slideEndSpeed = 15f;
    [SerializeField] private float slideFriction = 0.4f;
    [SerializeField] private float slideSteerAcceleration = 5f;
    [SerializeField] private float slideGravity = -60f;
    [Space]
    [Header("Dash")]
    [SerializeField] private float dashDuration = 0.3f;
    [SerializeField] private float dashBonusSpeed = 25f;

    [SerializeField] private AnimationCurve dashSpeedCurve = new AnimationCurve
    (
        new Keyframe(0f, 1f),
        new Keyframe(0.3f, 1f),
        new Keyframe(1f, 0f)
    );
    [SerializeField] private float dashCooldown = 1.2f;
    [Space]
    [Header("Grappling")]
    [SerializeField] private LayerMask grappleLayerMask;
    [SerializeField] private Transform grappleStart;
    [SerializeField] private float grappleMaxDistance = 40;
    [SerializeField] private float minRopeLength = 2f;
    [SerializeField] float grappleReelSpeed = 35f;
    [Range(0f, 1f)]
    [SerializeField] private float grappleGravityDamp = .6f;
    [Range(0f, 1f)]
    [SerializeField] float grapplePullWeight = 0.5f;
    [SerializeField] float predictionSphereRadius = 1.5f;
    [Space]
    [Header("Grappling Rope Anim")]
    [SerializeField] private float ropeExtendSpeed = 100f;
    [SerializeField] private float waveAmplitude = 5f;
    [SerializeField] private float waveFrequency = 7f;
    [SerializeField] private float waveAnimSpeed = 5f;
    [SerializeField] private int ropeSegments = 60;
    [Space]
    [Header("Wall Run")]
    [SerializeField] private Transform wallrunCheckTransform;
    [SerializeField] private float wallRunSpeed = 20f;
    [SerializeField] private float wallRunDuration = 1.5f;
    [SerializeField] private float wallRunGravity = -15f;
    [SerializeField] private float wallDetectionDistance = 1f;
    [SerializeField] private LayerMask wallRunLayerMask;
    [SerializeField] private float wallStickForce = 10f;
    [SerializeField] private float wallJumpOutwardForce = 26f;
    [SerializeField] private float wallJumpUpwardForce = 25f;
    [Space]
    [Header("Player height")]
    [SerializeField] private float standHeight = 2f;
    [SerializeField] private float crouchHeight = 1f;
    [SerializeField] private float crouchHeightResponse = 15f;

    [Range(0f, 1f)]
    [SerializeField] private float standCameraTargetHeight = 0.9f;
    [Range(0f, 1f)]
    [SerializeField] private float crouchCameraTargetHeight = .7f;
    [Range(0f, 1f)]
    [SerializeField] private float grappleHeight = 0.5f;

    private CharacterState _state;
    private CharacterState _lastState;
    private CharacterState _tempState;

    private Quaternion _requestedRotation;
    private Vector3 _requestedMovement;
    private bool _requestedJump;
    private bool _requestedSustainedJump;
    private bool _requestedCrouch;
    private bool _requestedCrouchInAir;

    private float _timeSinceUngrounded;
    private float _timeSinceJumpRequest;
    private bool _ungroundedDueToJump;
    private float _remainingJumps;
    private Collider[] _uncrouchOverlapResults;

    private bool _requestedDash;
    private bool _isDashing;
    private float _dashTimeRemaining;
    private float _dashCooldownRemaining;
    private Vector3 _dashDirection;
    private float _curMaxDashSpeed;
    private float _curMinDashSpeed;
    private bool hasDashedThisJump;

    private float _curMaxPlanarSpeed;

    private bool _requestedGrappling;
    private bool _isGrappling = false;
    private LineRenderer _lineRenderer;
    private Vector3 _grapplePoint;
    private float _ropeLength;
    private float _maxReelSpeed;
    private bool _reachedRopeLength;
    private float _curMinRopeLength;
    private RaycastHit _predictionHit;
    private bool _hasReleasedGrapplingButton;

    private float _currentRopeLength = 0f;
    private bool _ropeExtending = false;
    private bool _grappleInProgress = false;
    private Vector3 _waveDirection;

    private bool _isWallRunning = false;
    private float _wallRunTime = 0f;
    private Vector3 _wallNormal;

    public void Initialize()
    {
        _state.Stance = Stance.Stand;
        _lastState = _state;
        _uncrouchOverlapResults = new Collider[8];
        _remainingJumps = numberOfJumps;
        motor.CharacterController = this;
        _lineRenderer = GetComponent<LineRenderer>();
    }

    public void UpdateInput(CharacterInput input)
    {
        _requestedRotation = input.Rotation;
        _requestedMovement = new Vector3(input.Move.x, 0f, input.Move.y);
        // Avoid diagnoal speed up
        _requestedMovement = Vector3.ClampMagnitude(_requestedMovement, 1f);
        // Orient the input so it's relative to the direction player is facing
        _requestedMovement = input.Rotation * _requestedMovement;

        var wasRequestingJump = _requestedJump;
        _requestedJump = _requestedJump || input.Jump;
        if (_requestedJump && !wasRequestingJump)
        {
            _timeSinceJumpRequest = 0f;
        }
        _requestedSustainedJump = input.JumpSustain;

        var wasRequestingCrouch = _requestedCrouch;
        _requestedCrouch = input.Crouch switch
        {
            CrouchInput.Toggle => !_requestedCrouch,
            CrouchInput.None => _requestedCrouch,
            _ => _requestedCrouch
        };
        if (_requestedCrouch && !wasRequestingCrouch)
        {
            _requestedCrouchInAir = !_state.Grounded;
        }
        else if (!_requestedCrouch && wasRequestingCrouch)
        {
            _requestedCrouchInAir = false;
        }

        if (input.Dash
            && !hasDashedThisJump
            && _dashCooldownRemaining <= 0f
            && !_isDashing
            && !(_state.Stance is Stance.Crouch && motor.GroundingStatus.IsStableOnGround))
        {
            _requestedDash = true;
        }

        _requestedGrappling = input.Grapple switch
        {
            GrappleInput.Toggle => true,
            GrappleInput.None => false,
            _ => _requestedGrappling
        };

        // the 2 following checks are done to avoid perma grappling
        // if the button hasn't been release since the end of the last grapple
        if (!_hasReleasedGrapplingButton && !_requestedGrappling)
        {
            _hasReleasedGrapplingButton = true;
        }

        if (_requestedGrappling && !_hasReleasedGrapplingButton)
        {
            _requestedGrappling = false;
        }
    }

    public void UpdateBody(float deltaTime)
    {
        var currentHeight = motor.Capsule.height;
        var normalizedHeight = currentHeight / standHeight;
        var cameraTargetHeight = currentHeight *
            (
                _state.Stance is Stance.Stand
                    ? standCameraTargetHeight
                    : crouchCameraTargetHeight
            );

        var grappleStartHeight = currentHeight * grappleHeight;
        var wallrunCheckHeight = currentHeight * grappleHeight;
        var rootTargetScale = new Vector3(1f, normalizedHeight, 1f);

        // Animate camera moving to get it smoother
        cameraTarget.localPosition = Vector3.Lerp
            (
                a: cameraTarget.localPosition,
                b: new Vector3(0f, cameraTargetHeight, 0f),
                t: 1f - Mathf.Exp(-crouchHeightResponse * deltaTime)
            );

        grappleStart.localPosition = Vector3.Lerp
        (
            a: grappleStart.localPosition,
            b: new Vector3(grappleStart.localPosition.x, grappleStartHeight, grappleStart.localPosition.z),
            t: 1f - Mathf.Exp(-crouchHeightResponse * deltaTime)
        );

        wallrunCheckTransform.localPosition = Vector3.Lerp
        (
            a: wallrunCheckTransform.localPosition,
            b: new Vector3(wallrunCheckTransform.localPosition.x, wallrunCheckHeight, wallrunCheckTransform.localPosition.z),
            t: 1f - Mathf.Exp(-crouchHeightResponse * deltaTime)
        );

        root.localScale = Vector3.Lerp
            (
                a: root.localScale,
                b: rootTargetScale,
                t: 1f - Mathf.Exp(-crouchHeightResponse * deltaTime)
            );
    }

    public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
    {
        // Start dash if requested (not on slide or crouch except if air crouch)
        if (_requestedDash
            && _state.Stance is not Stance.Slide
            && _state.Stance is not Stance.Grapple
            && _state.Stance is not Stance.WallRun
            && !(_state.Stance is Stance.Crouch && motor.GroundingStatus.IsStableOnGround)
            && !hasDashedThisJump)
        {
            StartDashState();
        }

        if (!_state.Grounded && !_isWallRunning && !_isGrappling && !_isDashing)
        {
            if (CheckDistanceToGround() > 2.5f) // arbitrary value
            {
                if (CheckWall(out Vector3 wallNormal))
                {
                    StartWallRun(wallNormal);
                }
                else
                {
                    _state.Stance = Stance.Slide;
                }
            }

        }

        _state.Acceleration = Vector3.zero;
        if (_dashCooldownRemaining > 0f)
        {
            _dashCooldownRemaining -= deltaTime;
        }

        if (_isDashing)
        {
            _state.Stance = Stance.Dash;
            var fellFromGround = _lastState.Grounded && !_state.Grounded;
            if (fellFromGround)
            {
                if (_remainingJumps - 1 > 0)
                {
                    _remainingJumps--;
                }
            }
            _dashTimeRemaining -= deltaTime;

            float dashProgress = 1f - (_dashTimeRemaining / dashDuration);
            float speedFactor = dashSpeedCurve.Evaluate(dashProgress);
            float dashSpeed = Mathf.Lerp(_curMinDashSpeed, _curMaxDashSpeed, speedFactor);

            currentVelocity = motor.GetDirectionTangentToSurface
                        (
                            direction: _dashDirection,
                            surfaceNormal: motor.GroundingStatus.GroundNormal
                        ) * dashSpeed;

            if (_dashTimeRemaining <= 0f)
                EndDashState();

            return;
        }

        var wasInAir = !_lastState.Grounded;

        var wasGrappling = _isGrappling;
        if (_requestedGrappling)
        {
            if (!wasGrappling)
            {
                if (_predictionHit.point != Vector3.zero)
                {
                    StartGrappleState(_predictionHit, ref currentVelocity);
                }
            }
        }
        else
        {
            if (wasGrappling)
            {
                EndGrappleState();
            }
        }

        if (_isGrappling)
        {
            /*_state.Stance = Stance.Grapple;
            _requestedJump = false;*/
            _requestedDash = false;
            if (_state.Grounded)
                motor.ForceUnground(0.1f);
            _remainingJumps = numberOfJumps - 1; // Re add the double jump if already used
            Vector3 toAnchor = _grapplePoint - transform.position;
            Vector3 dirToAnchor = toAnchor.normalized;

            // Pull towards grapple point
            float distance = toAnchor.magnitude;
            Vector3 gravityForce = Vector3.ProjectOnPlane(Physics.gravity * grappleGravityDamp, dirToAnchor);

            if (!_reachedRopeLength && distance > _curMinRopeLength + .1f)
            {
                // Blend between grapple pull and character input direction
                Vector3 inputDir = _requestedMovement.normalized;
                Vector3 blendedDir = Vector3.Slerp(inputDir, dirToAnchor, grapplePullWeight).normalized;

                // Apply movement speed (reel speed or boost speed)
                var curReelSpeed = Mathf.Lerp(_maxReelSpeed, grappleReelSpeed, deltaTime);
                currentVelocity = blendedDir * curReelSpeed + gravityForce * deltaTime;
            }
            else
            {
                // End grapple state once we reach the min rope length
                EndGrappleState();
            }
        }

        if (_isWallRunning)
        {
            _state.Stance = Stance.WallRun;
            _wallRunTime += deltaTime;

            // Stick force
            currentVelocity -= _wallNormal * wallStickForce * deltaTime;

            // Forward movement along wall 
            Vector3 wallForward = Vector3.Cross(_wallNormal, motor.CharacterUp);
            wallForward = Vector3.Dot(wallForward, _requestedMovement) > 0f ? wallForward : -wallForward; // TODO FIX THIS TO STOP GRAPPLING IF NO FRONT INPUT

            // Speed falloff factor
            float speedFactor = Mathf.Clamp01(1f - (_wallRunTime / wallRunDuration));
            float forwardBoost = Mathf.Max(wallRunSpeed, currentVelocity.magnitude) * speedFactor;

            // Only add boost if current speed along wall is less
            float currentWallSpeed = Vector3.Dot(currentVelocity, wallForward);
            if (currentWallSpeed < forwardBoost)
            {
                float boost = forwardBoost - currentWallSpeed;
                currentVelocity += wallForward * boost;
            }

            // Gravity
            currentVelocity += motor.CharacterUp * wallRunGravity * deltaTime;

            // Cancel wallrun if speed too low or time expired
            if (_wallRunTime > wallRunDuration || currentWallSpeed <= 1f || !CheckWall(out _) || motor.GroundingStatus.IsStableOnGround)
            {
                EndWallRun();
            }

            if (_requestedJump)
            {
                _requestedJump = false;

                // 1. Get current planar (horizontal) velocity
                Vector3 planarVelocity = Vector3.ProjectOnPlane(currentVelocity, motor.CharacterUp);

                // 2. Compute wall jump vertical + outward force
                Vector3 jumpVertical = motor.CharacterUp * wallJumpUpwardForce;
                Vector3 jumpOutward = _wallNormal * wallJumpOutwardForce;

                Vector3 wallJumpForce = jumpVertical + jumpOutward;

                // 3. Combine: Keep planar velocity + force vertical boost
                currentVelocity = planarVelocity + wallJumpForce;

                EndWallRun();
            }
            return;
        }

        if (motor.GroundingStatus.IsStableOnGround) // On the ground
        {
            _timeSinceUngrounded = 0f;
            _ungroundedDueToJump = false;
            _remainingJumps = numberOfJumps;

            // Snap requested movement direction to the angle of the surface the char is on
            var groundedMovement = motor.GetDirectionTangentToSurface
                (
                    direction: _requestedMovement,
                    surfaceNormal: motor.GroundingStatus.GroundNormal
                ) * _requestedMovement.magnitude;

            if (wasInAir)
            {
                hasDashedThisJump = false;
            }

            // Start sliding
            {
                var moving = groundedMovement.sqrMagnitude > 0f;
                var crouching = _state.Stance is Stance.Crouch;
                var wasStanding = _lastState.Stance is Stance.Stand;
                if (moving && crouching && (wasStanding || wasInAir))
                {
                    _state.Stance = Stance.Slide;

                    // When landing on stable ground the character motor projects the velocity onto a flat ground plane
                    // See : KinematicCharacterMotor.HandleVelocityProjection()
                    // Fix by reprojecting ourselves
                    if (wasInAir)
                    {
                        currentVelocity = Vector3.ProjectOnPlane
                            (
                                vector: _lastState.Velocity,
                                planeNormal: motor.GroundingStatus.GroundNormal
                            );
                    }

                    var effectiveSlideStartSpeed = slideStartSpeed;
                    if (!_lastState.Grounded && !_requestedCrouchInAir)
                    {
                        effectiveSlideStartSpeed = 0f;
                        _requestedCrouchInAir = false;
                    }

                    var slideSpeed = Mathf.Max(effectiveSlideStartSpeed, currentVelocity.magnitude);
                    currentVelocity = motor.GetDirectionTangentToSurface
                        (
                            direction: currentVelocity,
                            surfaceNormal: motor.GroundingStatus.GroundNormal
                        ) * slideSpeed;
                }
            }

            // Move
            if (_state.Stance is Stance.Stand or Stance.Crouch)
            {
                var speed = _state.Stance is Stance.Stand
                    ? walkSpeed
                    : crouchSpeed;

                // Calculate speed and responsiveness of movement based on character's stance
                var response = _state.Stance is Stance.Stand
                    ? walkResponse
                    : crouchResponse;

                // Move along the ground in that direction
                var targetVelocity = groundedMovement * speed;
                var moveVelocity = Vector3.Lerp
                    (
                        a: currentVelocity,
                        b: targetVelocity,
                        t: 1f - Mathf.Exp(-response * deltaTime)
                    );

                _state.Acceleration = moveVelocity - currentVelocity;
                currentVelocity = moveVelocity;
            }
            // Continue sliding
            else if (_state.Stance is Stance.Slide)
            {
                // Friction
                currentVelocity -= currentVelocity * (slideFriction * deltaTime);

                // Slope
                {
                    var force = Vector3.ProjectOnPlane
                        (
                            vector: -motor.CharacterUp,
                            planeNormal: motor.GroundingStatus.GroundNormal
                        ) * slideGravity;
                    currentVelocity -= force * deltaTime;
                }

                // Steer
                {
                    // Target vel in player movement direction at current speed
                    var currentSpeed = currentVelocity.magnitude;
                    var targetVelocity = groundedMovement * currentVelocity.magnitude;
                    var steerVelocity = currentVelocity;
                    var steerForce = (targetVelocity - steerVelocity) * slideSteerAcceleration * deltaTime;

                    // Add steer force but clamp to avoid speed gains
                    steerVelocity += steerForce;
                    steerVelocity = Vector3.ClampMagnitude(steerVelocity, currentSpeed);

                    _state.Acceleration = (steerVelocity - currentVelocity) / deltaTime;

                    currentVelocity = steerVelocity;
                }

                // Stop
                if (currentVelocity.magnitude < slideEndSpeed)
                {
                    _state.Stance = Stance.Crouch;
                }
            }
        }
        else // In the air
        {
            _timeSinceUngrounded += deltaTime;
            var canCoyoteJump = _timeSinceUngrounded < coyoteTime && !_ungroundedDueToJump;
            if (!wasInAir && !_requestedJump && !canCoyoteJump) // TODO fix not removing jump when falling check condition
            {
                if (_remainingJumps - 1 > 0)
                {
                    _remainingJumps--;
                }
            }

            // Move
            if (_requestedMovement.sqrMagnitude > 0f)
            {
                var planarMovement = Vector3.ProjectOnPlane
                    (
                        vector: _requestedMovement,
                        planeNormal: motor.CharacterUp
                    ).normalized * _requestedMovement.magnitude;

                // Current velocity on movement plane
                var currentPlanarVelocity = Vector3.ProjectOnPlane
                    (
                        vector: currentVelocity,
                        planeNormal: motor.CharacterUp
                    );

                // Calculate movement force
                var movementForce = planarMovement * airAcceleration * deltaTime;

                // If moving slower than max air speed, treat movementForce as a simple steering force
                if (currentPlanarVelocity.magnitude < airSpeed)
                {
                    // Add it to the current planar velocity for target vel
                    var targetPlanarVelocity = currentPlanarVelocity + movementForce;

                    // Limit target velocity to air speed
                    targetPlanarVelocity = Vector3.ClampMagnitude(targetPlanarVelocity, airSpeed);

                    // Steer towards target velocity
                    movementForce = targetPlanarVelocity - currentPlanarVelocity;
                }
                // Else nerf movement force when in direction of the current planar velocity
                else if (Vector3.Dot(currentPlanarVelocity, movementForce) > 0f)
                {
                    // project movemenbt force onto the plane whose normal is the current planar velocity
                    var constrainedMovementForce = Vector3.ProjectOnPlane
                        (
                            vector: movementForce,
                            planeNormal: currentPlanarVelocity.normalized
                        );
                    movementForce = constrainedMovementForce;
                }

                // Prevent air climbing steep slopes
                if (motor.GroundingStatus.FoundAnyGround)
                {
                    if (Vector3.Dot(movementForce, currentVelocity + movementForce) > 0f)
                    {
                        // Calculate obstruction normal
                        var obstructionNormal = Vector3.Cross
                            (
                                motor.CharacterUp,
                                Vector3.Cross
                                    (
                                        motor.CharacterUp,
                                        motor.GroundingStatus.GroundNormal
                                    )
                            ).normalized;

                        // Project movement force onto obstruction plane
                        movementForce = Vector3.ProjectOnPlane(movementForce, obstructionNormal);
                    }
                }
                currentVelocity += movementForce;
            }

            // Gravity
            var effectiveGravity = gravity;
            var verticalSpeed = Vector3.Dot(currentVelocity, motor.CharacterUp);
            if (_requestedSustainedJump && verticalSpeed > 0f)
            {
                effectiveGravity *= jumpSustainGravity;
            }

            currentVelocity += motor.CharacterUp * effectiveGravity * deltaTime;
        }

        if (_requestedJump)
        {
            var grounded = motor.GroundingStatus.IsStableOnGround;
            var canCoyoteJump = _timeSinceUngrounded < coyoteTime && !_ungroundedDueToJump;
            if ((grounded || canCoyoteJump) || _remainingJumps > 0)
            {
                _requestedJump = false; // Unset jump request
                _requestedCrouch = false; // Request uncrouch
                _requestedCrouchInAir = false;

                motor.ForceUnground(time: 0.1f);
                _ungroundedDueToJump = true;

                // Set minimum vertical speed to the jump speed
                var currentVerticalSpeed = Vector3.Dot(currentVelocity, motor.CharacterUp);
                var targetVerticalSpeed = Mathf.Max(currentVerticalSpeed, jumpSpeed);
                currentVelocity += motor.CharacterUp * (targetVerticalSpeed - currentVerticalSpeed);
                _remainingJumps -= 1;
            }
            else
            {
                _timeSinceJumpRequest += deltaTime;

                // Defer jump request until coyote time has passed
                var canJumpLater = _timeSinceJumpRequest < coyoteTime;
                _requestedJump = canJumpLater;
            }
        }

        ApplySoftSpeedCap(ref currentVelocity);
    }


    private bool CheckWall(out Vector3 wallNormal)
    {
        wallNormal = Vector3.zero;
        RaycastHit hit;

        Vector3 origin = wallrunCheckTransform.transform.position;
        Vector3 left = -wallrunCheckTransform.transform.right;
        Vector3 right = wallrunCheckTransform.transform.right;

        bool leftHit = Physics.Raycast(origin, left, out hit, wallDetectionDistance, wallRunLayerMask);
        if (!_state.Grounded)
        {
            if (leftHit)
            {
                if (IsWallRunnable(hit.normal))
                {
                    wallNormal = hit.normal;
                    return true;
                }
            }

            bool rightHit = Physics.Raycast(origin, right, out hit, wallDetectionDistance, wallRunLayerMask);
            if (rightHit)
            {
                if (IsWallRunnable(hit.normal))
                {
                    wallNormal = hit.normal;
                    return true;
                }
            }
        }
        return false;
    }

    private bool IsWallRunnable(Vector3 normal)
    {
        float angle = Vector3.Angle(normal, motor.CharacterUp);
        return angle > 85f && angle < 95f; // Only near-vertical walls
    }


    private void StartWallRun(Vector3 wallNormal)
    {
        _isWallRunning = true;
        _wallNormal = wallNormal;
        _wallRunTime = 0f;
        _state.Stance = Stance.WallRun;
        _remainingJumps = 1;
        hasDashedThisJump = false;
        motor.ForceUnground(0.1f);

        // Reset vertical speed to prevent bouncing
        Vector3 planarVelocity = Vector3.ProjectOnPlane(motor.Velocity, motor.CharacterUp);
        motor.BaseVelocity = planarVelocity;
    }

    private void EndWallRun()
    {
        _isWallRunning = false;
        _state.Stance = Stance.Stand;
    }

    private float CheckDistanceToGround()
    {
        if (!Physics.Raycast(transform.position, Vector3.down, out var hit))
        {
            return float.PositiveInfinity;
        }

        return hit.distance;
    }
    private void ApplySoftSpeedCap(ref Vector3 currentVelocity)
    {
        // Soft cap planar velocity
        Vector3 planarVelocity = Vector3.ProjectOnPlane(currentVelocity, motor.CharacterUp);
        float planarSpeed = planarVelocity.magnitude;

        _curMaxPlanarSpeed = _isDashing ? maxPlanarSpeed + dashBonusSpeed : maxPlanarSpeed;

        if (planarSpeed > _curMaxPlanarSpeed)
        {
            float excessSpeed = planarSpeed - _curMaxPlanarSpeed;
            float dampFactor = Mathf.Clamp01(excessSpeed / _curMaxPlanarSpeed);
            float speedScale = Mathf.Lerp(1f, 0.5f, dampFactor);

            planarVelocity = planarVelocity.normalized * (_curMaxPlanarSpeed + excessSpeed * speedScale);
        }

        // Soft cap vertical velocity
        Vector3 verticalVelocity = Vector3.Project(currentVelocity, motor.CharacterUp);
        float verticalSpeed = verticalVelocity.magnitude;

        if (verticalSpeed > maxVerticalSpeed)
        {
            float excessSpeed = verticalSpeed - maxVerticalSpeed;
            float dampFactor = Mathf.Clamp01(excessSpeed / maxVerticalSpeed);
            float speedScale = Mathf.Lerp(1f, 0.5f, dampFactor);

            verticalVelocity = verticalVelocity.normalized * (maxVerticalSpeed + excessSpeed * speedScale);
        }

        currentVelocity = planarVelocity + verticalVelocity;
    }

    public void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
    {
        var forward = Vector3.ProjectOnPlane
        (
            _requestedRotation * Vector3.forward,
            motor.CharacterUp
        );

        if (forward != Vector3.zero)
        {
            currentRotation = Quaternion.LookRotation(forward, motor.CharacterUp);
        }
    }

    public void BeforeCharacterUpdate(float deltaTime)
    {
        _tempState = _state;
        // Crouch
        if (_requestedCrouch && _state.Stance is Stance.Stand)
        {
            _state.Stance = Stance.Crouch;
            motor.SetCapsuleDimensions
            (
                radius: motor.Capsule.radius,
                height: crouchHeight,
                yOffset: crouchHeight * 0.5f
            );
        }
    }

    public void AfterCharacterUpdate(float deltaTime)
    {
        // Uncrouch
        if (!_requestedCrouch 
            && _state.Stance is not Stance.Stand 
            && _state.Stance is not Stance.Dash 
            && _state.Stance is not Stance.Grapple
            && _state.Stance is not Stance.WallRun)
        {
            motor.SetCapsuleDimensions
            (
                radius: motor.Capsule.radius,
                height: standHeight,
                yOffset: standHeight * 0.5f
            );

            var pos = motor.TransientPosition;
            var rot = motor.TransientRotation;
            var mask = motor.CollidableLayers;
            if (motor.CharacterOverlap(pos, rot, _uncrouchOverlapResults, mask, QueryTriggerInteraction.Ignore) > 0)
            {
                _requestedCrouch = true;
                motor.SetCapsuleDimensions
                (
                    radius: motor.Capsule.radius,
                    height: crouchHeight,
                    yOffset: crouchHeight * 0.5f
                );
            }
            else
            {
                _state.Stance = Stance.Stand;
            }
        }

        if (_isWallRunning && motor.GroundingStatus.IsStableOnGround)
        {
            EndWallRun();
        }

        // Update state to reflect relevant motor properties
        _state.Grounded = motor.GroundingStatus.IsStableOnGround;
        _state.Velocity = motor.Velocity;
        // Update the _lastState to store the car state snapshot taken at the beginning of this char update
        _lastState = _tempState;
    }

    public bool IsColliderValidForCollisions(Collider coll)
    {
        return true;
    }

    public void OnDiscreteCollisionDetected(Collider hitCollider)
    {
    }

    public void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
    {
    }

    public void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
    {
    }

    public void PostGroundingUpdate(float deltaTime)
    {
        if (!motor.GroundingStatus.IsStableOnGround && _state.Stance is Stance.Slide)
        {
            _state.Stance = Stance.Crouch;
        }
    }

    public void ProcessHitStabilityReport(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, Vector3 atCharacterPosition, Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport)
    {
    }

    public void CheckGrapplePoints()
    {
        if (_isGrappling) return;

        RaycastHit sphereCastHit;
        Physics.SphereCast(cameraTransform.position, predictionSphereRadius, cameraTransform.forward, out sphereCastHit, grappleMaxDistance, grappleLayerMask);

        RaycastHit hit;
        Physics.Raycast(cameraTransform.position, cameraTransform.forward, out hit, grappleMaxDistance, grappleLayerMask);

        _predictionHit = hit.point == Vector3.zero ? sphereCastHit : hit;
    }

    private void StartGrappleState(RaycastHit hit, ref Vector3 currentVelocity)
    {
        // Make sure we start grapple state only once
        if (_grappleInProgress) return;
        _grappleInProgress = true;

        _grapplePoint = hit.point;
        
        _ropeLength = Vector3.Distance(grappleStart.transform.position, _grapplePoint);
        _curMinRopeLength = Mathf.Min(_ropeLength, minRopeLength);

        _maxReelSpeed = Mathf.Max(grappleReelSpeed, currentVelocity.magnitude);

        _currentRopeLength = 0f;
        _ropeExtending = true;

        Vector3 ropeDir = (_grapplePoint - grappleStart.position).normalized;
        //Get any vector not aligned with ropeDir
        Vector3 randomVec = Random.insideUnitSphere;
        if (Mathf.Abs(Vector3.Dot(ropeDir, randomVec)) > 0.95f)
        {
            randomVec = Vector3.right;
        }
        //Cross product for perpendicular
        Vector3 waveDir = Vector3.Cross(ropeDir, randomVec).normalized;
        //Project waveDir onto horizontal plane (Y = 0)
        waveDir.y = 0f;
        _waveDirection = waveDir.normalized;
    }

    private void EndGrappleState()
    {
        _isGrappling = false;
        _requestedGrappling = false;
        _reachedRopeLength = false;
        _lineRenderer.positionCount = 0;
        _lineRenderer.enabled = false;
        _state.Stance = Stance.Stand;
        _hasReleasedGrapplingButton = false;
        _grappleInProgress = false;
        _ropeExtending = false;
    }

    public void DrawRope()
    {
        if (!_requestedGrappling)
        {
            EndGrappleState();
            return;
        }

        _lineRenderer.enabled = true;
        _lineRenderer.positionCount = ropeSegments + 1;

        Vector3 start = grappleStart.position;
        Vector3 end = _grapplePoint;
        float fullLength = Vector3.Distance(start, end);

        // Extend rope length
        if (_ropeExtending)
        {
            _currentRopeLength += ropeExtendSpeed * Time.deltaTime;
            if (_currentRopeLength >= fullLength)
            {
                _currentRopeLength = fullLength;
                _ropeExtending = false;
                StartGrapplePull();
            }
        }

        // Wave dir
        float maxT = Mathf.Clamp01(_currentRopeLength / fullLength);
        for (int i = 0; i <= ropeSegments; i++)
        {
            float t = (float)i / ropeSegments;
            float useT = Mathf.Min(t, maxT); // Keeps rope within extended length
            Vector3 point = Vector3.Lerp(start, end, useT);

            // Animate wave only during extension
            if (_ropeExtending && t < maxT)
            {
                float wave = Mathf.Sin((t * waveFrequency + Time.time * waveAnimSpeed) * Mathf.PI * 2f)
                             * waveAmplitude * (1 - t);
                float fadeFromStart = Mathf.SmoothStep(0f, .3f, t); // fade in from hand
                point += _waveDirection * wave * fadeFromStart;
            }
            _lineRenderer.SetPosition(i, point);
        }
    }

    private void StartGrapplePull()
    {
        // Now actually start pulling the player (grapple active now)
        _isGrappling = true;
        _state.Stance = Stance.Grapple;
    }

    private void StartDashState()
    {
        _state.Stance = Stance.Dash;
        Vector3 currentPlanarVel = Vector3.ProjectOnPlane(motor.Velocity, motor.CharacterUp);
        _curMaxDashSpeed = currentPlanarVel.magnitude + dashBonusSpeed;
        _curMinDashSpeed = currentPlanarVel.magnitude;
        _isDashing = true;
        _dashTimeRemaining = dashDuration;
        _dashCooldownRemaining = dashCooldown;
        _requestedDash = false;
        if (_isGrappling)
        {
            _isGrappling = false;
            EndGrappleState();
        }
        // Set dash direction (use input direction or facing forward)
        Vector3 lookForward = Vector3.ProjectOnPlane(_requestedRotation * Vector3.forward, motor.CharacterUp).normalized;
        _dashDirection = lookForward;

        motor.ForceUnground(.1f);

        if (!motor.GroundingStatus.IsStableOnGround)
            hasDashedThisJump = true;
    }

    private void EndDashState()
    {
        _isDashing = false;
        _state.Stance = Stance.Stand;
    }

    public Transform GetCameraTarget() => cameraTarget;
    public CharacterState GetState() => _state;
    public CharacterState GetLastState() => _lastState;
    public float GetJumpsRemaining() => _remainingJumps;
    public float GetDashCurrentCooldown() => _dashCooldownRemaining;
    public float GetDashMaxCooldown() => dashCooldown;
    public bool GetIsGrappling() => _isGrappling;
    public bool GetIsRopeExtending() => _ropeExtending;
    public bool GetGrapplePredictionHitPoint() => _predictionHit.point != Vector3.zero && _hasReleasedGrapplingButton;
    public bool GetCanDash() => _state.Stance is not Stance.Slide
            && !(_state.Stance is Stance.Crouch && motor.GroundingStatus.IsStableOnGround)
            && !(_state.Stance is Stance.Grapple)
            && !(_state.Stance is Stance.WallRun)
            && !hasDashedThisJump
            && !(_dashCooldownRemaining > 0f);
    public bool GetIsDashing() => _isDashing;
    public Vector3 GetWallRunWallNormal() => _wallNormal;

    public void SetPosition(Vector3 position, bool killVelocity = true)
    {
        motor.SetPosition(position);
        if (killVelocity)
        {
            motor.BaseVelocity = Vector3.zero;
        }
    }

}
