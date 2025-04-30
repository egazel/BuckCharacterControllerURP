using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering;

public class Player : MonoBehaviour
{
    [SerializeField] private PlayerCharacter playerCharacter;
    [SerializeField] private PlayerCamera playerCamera;
    [Space]
    [Header("Animation & Post")]
    [SerializeField] private CameraSpring cameraSpring;
    [SerializeField] private CameraLean cameraLean;
    [SerializeField] private Volume volume;
    [SerializeField] private StanceVignette stanceVignette;
    [Header("Debug texts")]
    [SerializeField] private TextMeshProUGUI groundedTMP;
    [SerializeField] private TextMeshProUGUI stanceTMP;
    [SerializeField] private TextMeshProUGUI canDashTMP;
    [SerializeField] private TextMeshProUGUI isDashingTMP;
    [SerializeField] private TextMeshProUGUI jumpsRemainingTMP;
    [SerializeField] private TextMeshProUGUI velocityTmp;


    private PlayerInputActions _inputActions;

    void Start()
    {
        Cursor.lockState = CursorLockMode.Locked;

        _inputActions = new PlayerInputActions();
        _inputActions.Enable();

        playerCharacter.Initialize();
        playerCamera.Initialize(playerCharacter.GetCameraTarget());

        cameraSpring.Initialize();
        cameraLean.Initialize();

        stanceVignette.Initialize(volume.profile);
    }

    private void OnDestroy()
    {
        _inputActions.Dispose();
    }

    void Update()
    {
        var deltaTime = Time.deltaTime;
        var input = _inputActions.Gameplay;

        // Get camera input/target and update its rotation/position
        var cameraInput = new CameraInput { Look = input.Look.ReadValue<Vector2>() };
        playerCamera.UpdateRotation(cameraInput);

        // Get character input and update it
        var characterInput = new CharacterInput
        {
            Rotation = playerCamera.transform.rotation,
            Move = input.Move.ReadValue<Vector2>(),
            Jump = input.Jump.WasPressedThisFrame(),
            JumpSustain = input.Jump.IsPressed(),
            Dash = input.Dash.WasPressedThisFrame(),
            Crouch = input.Crouch.WasPressedThisFrame()
                ? CrouchInput.Toggle
                : CrouchInput.None
        };
        playerCharacter.UpdateInput(characterInput);
        playerCharacter.UpdateBody(deltaTime);

#if UNITY_EDITOR
        if (Keyboard.current.tKey.wasPressedThisFrame)
        {
            var ray = new Ray(playerCamera.transform.position, playerCamera.transform.forward);
            if (Physics.Raycast(ray, out var hit))
            {
                Teleport(hit.point);
            }
        }
        #endif
    }

    private void LateUpdate()
    {
        var deltaTime = Time.deltaTime;
        var state = playerCharacter.GetState();

        groundedTMP.SetText("Grounded : " + state.Grounded.ToString());
        stanceTMP.SetText("Stance : " + state.Stance.ToString());
        velocityTmp.SetText("Velocity : " + Mathf.Round(state.Velocity.magnitude).ToString());
        canDashTMP.SetText("Can dash : " + playerCharacter.GetCanDash().ToString());
        isDashingTMP.SetText("Is Dashing : " + playerCharacter.GetIsDashing().ToString());
        jumpsRemainingTMP.SetText("Jumps remaining : " + playerCharacter.GetJumpsRemaining().ToString());
        
        // Update camera position in LateUpdate to avoid lagging behind 1 frame and seeing the player
        var cameraTarget = playerCharacter.GetCameraTarget();
        playerCamera.UpdatePosition(cameraTarget);

        cameraSpring.UpdateSpring(deltaTime, cameraTarget.up);
        cameraLean.UpdateLean
            (
                deltaTime,
                state.Stance is Stance.Slide,
                state.Acceleration,
                cameraTarget.up
            );

        playerCamera.UpdateFOV(state.Velocity.magnitude, deltaTime);

        stanceVignette.UpdateVignette(deltaTime, state.Stance);

        playerCamera.UpdateSpeedLinesVFX(state.Velocity.magnitude);
    }

    public void Teleport(Vector3 position)
    {
        playerCharacter.SetPosition(position);
    }
}
