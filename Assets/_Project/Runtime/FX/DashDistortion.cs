using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class DashDistortion : MonoBehaviour
{
    private VolumeProfile _profile;
    private LensDistortion _lensDistortion;

    private float _intensity;
    private float _timer;
    private bool _playing;

    [Header("Distortion Settings")]
    [SerializeField] private float maxIntensity = -.8f;
    [SerializeField] private float effectDuration = 0.2f;
    [SerializeField] private float lerpSpeed = 10f;

    public void Initialize(VolumeProfile profile)
    {
        _profile = profile;

        // Get lens distortion override
        if (!_profile.TryGet(out _lensDistortion))
        {
            Debug.LogWarning("LensDistortion not found in profile!");
        }

        _intensity = 0f;
        _lensDistortion.intensity.value = 0f;
        _playing = false;
    }

    public void TriggerDash()
    {
        _timer = 0f;
        _playing = true;
    }

    public void UpdateDashDistortion(float deltaTime)
    {
        if (!_lensDistortion)
            return;

        if (_playing)
        {
            _timer += deltaTime;

            // Compute progress
            float t = Mathf.Clamp01(_timer / effectDuration);

            // Ease-out curve
            float curve = 1f - Mathf.Pow(1f - t, 3f); // Cubic ease-out

            // Lerp target intensity
            float target = Mathf.Lerp(maxIntensity, 0f, curve);

            _intensity = Mathf.Lerp(_intensity, target, lerpSpeed * deltaTime);
            _lensDistortion.intensity.value = _intensity;

            // Stop playing when finished
            if (t >= 1f && Mathf.Abs(_intensity) < 0.01f)
            {
                _playing = false;
                _intensity = 0f;
                _lensDistortion.intensity.value = 0f;
            }
        }
    }
}