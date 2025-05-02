using UnityEngine;
using UnityEngine.UIElements;
using UnityEngine.VFX;

public struct CameraInput
{
    public Vector2 Look;
}

public class PlayerCamera : MonoBehaviour
{
    [Header("Mouse sensitivity")]
    [SerializeField] private float sensitivity = 0.08f;
    [Space]
    [Header("Dynamic Speed FOV")]
    [SerializeField] private Camera playerCam;
    [SerializeField] private float baseFOV = 70f;
    [SerializeField] private float maxFOV = 90f;
    [SerializeField] private float fovSpeedMultiplier = 0.15f; // How much FOV increases per speed unit
    [SerializeField] private float fovLerpSpeed = 4f;

    [Space]
    [Header("Speed Lines VFX")]
    [SerializeField] private VisualEffect speedLinesVFX;
    [SerializeField] private Vector2 speed = new Vector2(25f, 150f);
    [SerializeField] private Vector2 spawnRate = new Vector2(0f, 300f);
    [SerializeField] private Vector2 outRate = new Vector2(4f, 8f);
    [SerializeField] private Vector2 xScaleRange = new Vector2(8f, 50f);

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
        _eulerAngles.x = Mathf.Clamp(_eulerAngles.x, -89f, 89f);
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

    public void UpdateSpeedLinesVFX(float currentSpeed)
    {
        float speedFactor = Mathf.InverseLerp(speed.x, speed.y, currentSpeed);

        float curSpawnRate = Mathf.Lerp(spawnRate.x, spawnRate.y, speedFactor);
        speedLinesVFX.SetFloat("SpawnRate", curSpawnRate);

        float curSpeedOutRate = Mathf.Lerp(outRate.x, outRate.y, speedFactor);
        speedLinesVFX.SetFloat("OutSpeed", curSpeedOutRate);

        float newY = Mathf.Lerp(xScaleRange.x, xScaleRange.y, speedFactor);
        speedLinesVFX.SetFloat("ScaleRangeY", newY);
    }
}
