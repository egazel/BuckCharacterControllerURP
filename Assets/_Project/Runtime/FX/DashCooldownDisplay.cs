using UnityEngine;
using UnityEngine.UI;

public class DashCooldownDisplay : MonoBehaviour
{
    [SerializeField] private PlayerCharacter playerCharacter;
    [SerializeField] private Image _cdImage;
    [SerializeField] private Color32 _backgroundColor;
    [SerializeField] private Color32 _activeColor;
    [SerializeField] private Color32 _notReadyColor;

    private Image _dynamicImageInstance;
    private Image _dynamicImageComponent;

    public void Initialize()
    {
        if (_cdImage != null)
            _cdImage.gameObject.SetActive(true);
            _cdImage.GetComponent<Image>().color = _backgroundColor;
            _cdImage.fillAmount = 1f;

            _dynamicImageInstance = Instantiate(_cdImage, _cdImage.transform.parent);
            _dynamicImageInstance.transform.SetPositionAndRotation(_cdImage.transform.position, _cdImage.transform.rotation);
            _dynamicImageInstance.transform.localScale = _cdImage.transform.localScale;
            _dynamicImageComponent = _dynamicImageInstance.GetComponent<Image>();
    }

    public void UpdateDashCooldownDisplay(float cooldown, bool canDash, float maxCooldown)
    {
        if (_dynamicImageInstance == null)
            return;

        // Hide if dash is ready
        if (cooldown <= 0f)
        {
            _cdImage.gameObject.SetActive(false);
            if (!canDash)
            {
                _dynamicImageInstance.fillAmount = 1f;
                _dynamicImageComponent.color = _notReadyColor;
                _dynamicImageInstance.gameObject.SetActive(true);
            }
            else
            {
                _dynamicImageInstance.fillAmount = 0f;
                _dynamicImageComponent.color = _activeColor;
            }
            return;
        }
        _cdImage.gameObject.SetActive(true);

        /*_cdImage.gameObject.SetActive(true);*/
        _dynamicImageComponent.color = _activeColor;

        // Clamp fill between 0 and 1
        float fillAmount = Mathf.Clamp01(1f - (cooldown / maxCooldown));
        _dynamicImageInstance.fillAmount = fillAmount;
    }
}
