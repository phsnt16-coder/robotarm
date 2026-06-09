using UnityEngine;
using UnityEngine.InputSystem;

public class JointWorldPositionPrinter : MonoBehaviour
{
    public Transform baseJoint;
    public Transform shoulderJoint;
    public Transform elbowJoint;
    public Transform gripperRotationJoint;
    public Transform gripperJoint;
    public Transform gripCenter;

    public float unityUnitToMm = 1f;

    void Update()
    {
        Keyboard keyboard = Keyboard.current;

        if (keyboard == null)
        {
            return;
        }

        if (keyboard.pKey.wasPressedThisFrame)
        {
            PrintWorldPositions();
        }
    }

    [ContextMenu("Print World Positions")]
    public void PrintWorldPositions()
    {
        PrintPosition("BASE_JOINT", baseJoint);
        PrintPosition("SHOULDER_JOINT", shoulderJoint);
        PrintPosition("ELBOW_JOINT", elbowJoint);
        PrintPosition("GRIPPER_ROTATION_JOINT", gripperRotationJoint);
        PrintPosition("GRIPPER_JOINT", gripperJoint);
        PrintPosition("GRIP_CENTER", gripCenter);

        PrintDistance("BASE_TO_SHOULDER", baseJoint, shoulderJoint);
        PrintDistance("SHOULDER_TO_ELBOW", shoulderJoint, elbowJoint);
        PrintDistance("ELBOW_TO_GRIPPER_ROTATION", elbowJoint, gripperRotationJoint);
        PrintDistance("GRIPPER_ROTATION_TO_GRIPPER", gripperRotationJoint, gripperJoint);
        PrintDistance("GRIPPER_TO_GRIP_CENTER", gripperJoint, gripCenter);
    }

    void PrintPosition(string label, Transform target)
    {
        if (target == null)
        {
            Debug.LogError(label + " 연결 안 됨");
            return;
        }

        Debug.Log(label + " WORLD POSITION = " + target.position.ToString("F4"));
    }

    void PrintDistance(string label, Transform from, Transform to)
    {
        if (from == null || to == null)
        {
            Debug.LogError(label + " 거리 계산 실패");
            return;
        }

        float distanceUnity = Vector3.Distance(from.position, to.position);
        float distanceMm = distanceUnity * unityUnitToMm;

        Debug.Log(label + " DISTANCE = " + distanceMm.ToString("F2") + " mm");
    }
}
