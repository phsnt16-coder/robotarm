using UnityEngine;
using UnityEngine.InputSystem;

public class CameraSwitcher : MonoBehaviour
{
    public Camera frontCamera;

    public Camera leftCamera;

    public Camera rightCamera;

    public Camera topCamera;

    public Camera cameraTop;

    private Camera currentCamera;

    private string currentCameraName = "None";

    void Start()
    {
        ActivateCamera(
            frontCamera,
            "FRONT_Camera"
        );
    }

    void Update()
    {
        Keyboard keyboard = Keyboard.current;

        if (keyboard == null)
        {
            return;
        }

        if (keyboard.zKey.wasPressedThisFrame)
        {
            ActivateCamera(
                frontCamera,
                "FRONT_Camera"
            );
        }

        if (keyboard.xKey.wasPressedThisFrame)
        {
            ActivateCamera(
                leftCamera,
                "LEFT_Camera"
            );
        }

        if (keyboard.cKey.wasPressedThisFrame)
        {
            ActivateCamera(
                rightCamera,
                "RIGHT_Camera"
            );
        }

        if (keyboard.vKey.wasPressedThisFrame)
        {
            ActivateCamera(
                topCamera,
                "TOP_Camera"
            );
        }

        if (keyboard.bKey.wasPressedThisFrame)
        {
            ActivateCamera(
                cameraTop,
                "Camera_Top"
            );
        }

        if (keyboard.spaceKey.wasPressedThisFrame)
        {
            Debug.Log(
                "CURRENT CAMERA : " +
                currentCameraName
            );
        }
    }

    void ActivateCamera(
        Camera targetCamera,
        string cameraName
    )
    {
        DisableCamera(frontCamera);

        DisableCamera(leftCamera);

        DisableCamera(rightCamera);

        DisableCamera(topCamera);

        DisableCamera(cameraTop);

        if (targetCamera == null)
        {
            Debug.LogError(
                cameraName +
                " 연결되지 않았습니다."
            );

            return;
        }

        targetCamera.gameObject.SetActive(true);

        currentCamera = targetCamera;

        currentCameraName = cameraName;

        Debug.Log(
            "CAMERA SWITCHED : " +
            currentCameraName
        );
    }

    void DisableCamera(Camera cam)
    {
        if (cam != null)
        {
            cam.gameObject.SetActive(false);
        }
    }
}
