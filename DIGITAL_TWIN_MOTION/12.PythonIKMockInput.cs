using UnityEngine;
using UnityEngine.InputSystem;

public class PythonIKMockInput : MonoBehaviour
{
    public RobotArmAutoSequence robotArmAutoSequence;

    [Header("Mock Pick IK")]
    public float pickBaseY = 0f;
    public float pickShoulderZ = 90f;
    public float pickElbowZ = -67f;
    public float pickGripperRotationZ = -25f;

    [Header("Mock Place IK")]
    public float placeBaseY = -90f;
    public float placeShoulderZ = 90f;
    public float placeElbowZ = -60f;
    public float placeGripperRotationZ = -25f;

    void Update()
    {
        Keyboard keyboard = Keyboard.current;

        if (keyboard == null)
        {
            return;
        }

        if (keyboard.iKey.wasPressedThisFrame)
        {
            SendMockPickIK();
        }

        if (keyboard.oKey.wasPressedThisFrame)
        {
            SendMockPlaceIK();
        }
    }

    void SendMockPickIK()
    {
        if (robotArmAutoSequence == null)
        {
            return;
        }

        IKAnglePacket packet =
            new IKAnglePacket(
                pickBaseY,
                pickShoulderZ,
                pickElbowZ,
                pickGripperRotationZ
            );

        robotArmAutoSequence.SetPickIKAngles(packet);
    }

    void SendMockPlaceIK()
    {
        if (robotArmAutoSequence == null)
        {
            return;
        }

        IKAnglePacket packet =
            new IKAnglePacket(
                placeBaseY,
                placeShoulderZ,
                placeElbowZ,
                placeGripperRotationZ
            );

        robotArmAutoSequence.SetPlaceIKAngles(packet);
    }
}
