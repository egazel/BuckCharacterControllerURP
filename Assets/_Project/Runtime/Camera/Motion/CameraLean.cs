using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraLean : MonoBehaviour
{
    [SerializeField] private float attackDamping = 0.4f;
    [SerializeField] private float decayDamping = 0.2f;
    [SerializeField] private float walkStrength = 0.065f;
    [SerializeField] private float slideStrength = 0.1f;
    [SerializeField] private float strengthResponse = 4f;
    [Header(" WallRun Lean")]
    [SerializeField] private float wallRunLeanAngle = 15f;
    [SerializeField] private float wallRunstrengthResponse = 8f;
    private Vector3 _dampedAcceleration;
    private Vector3 _dampedAccelerationVel;
    private float _smoothStrength;
    private float _currentWallRunLean;
    public void Initialize()
    {
        _smoothStrength = walkStrength;
    }

    public void UpdateLean(float deltaTime, Stance stance, Vector3 acceleration, Vector3 wallNormal, Vector3 up)
    {
        // 1. Only use lateral acceleration for slide lean (sideways only)
        var lateralAcceleration = Vector3.ProjectOnPlane(acceleration, up);
        lateralAcceleration = Vector3.Project(lateralAcceleration, transform.right);

        var damping = lateralAcceleration.magnitude > _dampedAcceleration.magnitude
            ? attackDamping
            : decayDamping;

        _dampedAcceleration = Vector3.SmoothDamp(
            current: _dampedAcceleration,
            target: lateralAcceleration,
            currentVelocity: ref _dampedAccelerationVel,
            smoothTime: damping,
            maxSpeed: float.PositiveInfinity,
            deltaTime: deltaTime
        );

        var targetStrength = stance is Stance.Slide ? slideStrength : walkStrength;
        _smoothStrength = Mathf.Lerp(_smoothStrength, targetStrength, 1f - Mathf.Exp(-strengthResponse * deltaTime));

        Quaternion slideLeanRot = Quaternion.identity;
        if (_dampedAcceleration.sqrMagnitude > 0.0001f)
        {
            // Get signed lateral accel (+right, -left)
            float signedLateral = Vector3.Dot(_dampedAcceleration, transform.right);

            float slideLeanAngle = -signedLateral * _smoothStrength;
            slideLeanRot = Quaternion.AngleAxis(slideLeanAngle, Vector3.forward); // Always roll sideways
        }


        // Wallrun lean rotation
        float wallRunTargetLean = 0f;
        if (stance is Stance.WallRun)
        {
            float side = Vector3.Dot(Vector3.Cross(up, wallNormal), transform.forward) > 0f ? 1f : -1f;
            wallRunTargetLean = side * wallRunLeanAngle;
        }

        // Smooth wallrun lean
        _currentWallRunLean = Mathf.Lerp(_currentWallRunLean, wallRunTargetLean, 1f - Mathf.Exp(-wallRunstrengthResponse * deltaTime));
        Quaternion wallRunLeanRot = Quaternion.AngleAxis(_currentWallRunLean, Vector3.forward);

        // 5. Combined rotation
        transform.localRotation = wallRunLeanRot * slideLeanRot;
    }

}
