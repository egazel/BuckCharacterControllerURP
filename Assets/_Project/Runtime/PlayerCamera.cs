using UnityEngine;

public struct CameraInput
{
    public Vector2 Look;
}

public class PlayerCamera : MonoBehaviour
{
    [SerializeField] private float sensitivity = 0.08f;
    [Header("Speed based FOV settings")]
    [SerializeField] private Camera playerCam;
    [SerializeField] private float baseFOV = 70f;
    [SerializeField] private float maxFOV = 85f;
    [SerializeField] private float fovSpeedMultiplier = 0.1f; // How much FOV increases per speed unit
    [SerializeField] private float fovLerpSpeed = 4f;

    private Vector3 _eulerAngles;
    public void Initialize(Transform target)
    {
        transform.position = target.position;
        transform.rotation = target.rotation;

        transform.eulerAngles = _eulerAngles = target.eulerAngles;
    }

    public void UpdateRotation(CameraInput input)
    {
        _eulerAngles += new Vector3(-input.Look.y, input.Look.x) * sensitivity;
        transform.eulerAngles = _eulerAngles;
    }

    public void UpdatePosition(Transform target)
    {
        transform.position = target.position;
    }
    public void UpdateFOV(float currentSpeed, float deltaTime)
    {
        float targetFOV = baseFOV + (currentSpeed * fovSpeedMultiplier);
        targetFOV = Mathf.Min(targetFOV, maxFOV);

        playerCam.fieldOfView = Mathf.Lerp(
            playerCam.fieldOfView,
            targetFOV,
            1f - Mathf.Exp(-fovLerpSpeed * deltaTime)
        );
    }
}
