using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class StanceVignette : MonoBehaviour
{
    [SerializeField] private float min = 0.1f;
    [SerializeField] private float max = 0.45f;
    [SerializeField] private float response = 10f;

    private VolumeProfile _profile;
    private Vignette _vignette;

    public void Initialize(VolumeProfile profile)
    {
        _profile = profile;
        if (!profile.TryGet(out _vignette))
        {
            _vignette = profile.Add<Vignette>();
        }
        _vignette.intensity.Override(min);
    }

    public void UpdateVignette(float deltaTime, Stance stance)
    {
        var targetIntensity = stance is Stance.Crouch || stance is Stance.Slide ? max : min;
        _vignette.intensity.value = Mathf.Lerp
            (
                a: _vignette.intensity.value,
                b: targetIntensity,
                t: 1f - Mathf.Exp(-response * deltaTime)
            );
    }
}
