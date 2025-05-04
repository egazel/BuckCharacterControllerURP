using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;

public class DashCooldownDisplay : MonoBehaviour
{
    [SerializeField] private PlayerCharacter playerCharacter;
    [SerializeField] private Image _cdImage;
    // Start is called before the first frame update
    public void Initialize()
    {
        if (_cdImage != null)
            _cdImage.gameObject.SetActive(false);
    }

    public void UpdateDashCooldownDisplay(float cooldown)
    {
        if (_cdImage == null)
            return;

        // Hide if dash is ready
        if (cooldown <= 0f)
        {
            _cdImage.gameObject.SetActive(false);
            return;
        }

        // Show and update fill
        _cdImage.gameObject.SetActive(true);

        // Clamp fill between 0 and 1
        float fillAmount = Mathf.Clamp01(cooldown);
        _cdImage.fillAmount = fillAmount;
    }
}
