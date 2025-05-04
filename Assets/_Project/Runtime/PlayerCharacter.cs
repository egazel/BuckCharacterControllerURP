using UnityEngine;
using KinematicCharacterController;

public enum CrouchInput
{
    None, Toggle
}

public enum Stance
{
    Stand, Crouch, Slide, Dash
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
}

public class PlayerCharacter : MonoBehaviour, ICharacterController
{
    [SerializeField] private KinematicCharacterMotor motor;
    [SerializeField] private Transform root;
    [SerializeField] private Transform cameraTarget;
    [Space]
    [Header("Walk/Crouch speed")]
    [SerializeField] private float walkSpeed = 20f;
    [SerializeField] private float crouchSpeed = 7f;
    [SerializeField] private float walkResponse = 25f;
    [SerializeField] private float crouchResponse = 20f;
    [SerializeField] float maxPlanarSpeed = 90f;
    [SerializeField] float maxVerticalSpeed = 100f;
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
    [SerializeField] private float slideStartSpeed = 25f;
    [SerializeField] private float slideEndSpeed = 15f;
    [SerializeField] private float slideFriction = 0.4f;
    [SerializeField] private float slideSteerAcceleration = 5f;
    [SerializeField] private float slideGravity = -90f;
    [Space]
    [Header("Dash")]
    [SerializeField] private float dashDuration = 0.3f;
    [SerializeField] private float dashBonusSpeed = 30f;

    [SerializeField] private AnimationCurve dashSpeedCurve = new AnimationCurve
    (
        new Keyframe(0f, 1f),
        new Keyframe(0.3f, 1f),
        new Keyframe(1f, 0f)
    );
    [SerializeField] private float dashCooldown = 1.2f;

    [Space]
    [Header("Player height")]
    [SerializeField] private float standHeight = 2f;
    [SerializeField] private float crouchHeight = 1f;
    [SerializeField] private float crouchHeightResponse = 15f;

    [Range(0f, 1f)]
    [SerializeField] private float standCameraTargetHeight = 0.9f;
    [Range(0f, 1f)]
    [SerializeField] private float crouchCameraTargetHeight = .7f;

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

    public void Initialize()
    {
        _state.Stance = Stance.Stand;
        _lastState = _state;
        _uncrouchOverlapResults = new Collider[8];
        _remainingJumps = numberOfJumps;
        motor.CharacterController = this;
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
        var rootTargetScale = new Vector3(1f, normalizedHeight, 1f);

        // Animate camera moving to get it smoother
        cameraTarget.localPosition = Vector3.Lerp
            (
                a: cameraTarget.localPosition,
                b: new Vector3(0f, cameraTargetHeight, 0f),
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
        _state.Acceleration = Vector3.zero;
        if (_dashCooldownRemaining > 0f)
        {
            _dashCooldownRemaining -= deltaTime;
        }

        // Start dash if requested (not on slide or crouch except if air crouch)
        if (_requestedDash 
            && _state.Stance is not Stance.Slide 
            && !(_state.Stance is Stance.Crouch && motor.GroundingStatus.IsStableOnGround)
            && !hasDashedThisJump)
        {
            StartDashState();
        }

        if (_isDashing)
        {
            _state.Stance = Stance.Dash; // TODO Find where Stance is being overriden during dash 
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
        if (!_requestedCrouch && _state.Stance is not Stance.Stand && _state.Stance is not Stance.Dash)
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

    public bool GetCanDash() => _state.Stance is not Stance.Slide
            && !(_state.Stance is Stance.Crouch && motor.GroundingStatus.IsStableOnGround)
            && !hasDashedThisJump
            && !(_dashCooldownRemaining > 0f);
    public bool GetIsDashing() => _isDashing;

    public void SetPosition(Vector3 position, bool killVelocity = true)
    {
        motor.SetPosition(position);
        if (killVelocity)
        {
            motor.BaseVelocity = Vector3.zero;
        }
    }

}
