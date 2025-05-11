using UnityEngine;
using UnityEngine.UI;

public class GrappleAbilityDisplay : MonoBehaviour
{
    [SerializeField] private PlayerCharacter playerCharacter;
    [Header("Can Grapple Display")]
    [SerializeField] private Image _dashDisplayImage;
    [SerializeField] private Color32 _dashDisplayColor;

    public void Initialize()
    {
        if (_dashDisplayImage != null)
            _dashDisplayImage.gameObject.SetActive(false);
        _dashDisplayImage.GetComponent<Image>().color = _dashDisplayColor;
        _dashDisplayImage.fillAmount = 1f;
    }

    public void UpdateGrappleAbilityDisplay(bool canGrapple)
    {
        _dashDisplayImage.gameObject.SetActive(canGrapple);
       
    }
}
