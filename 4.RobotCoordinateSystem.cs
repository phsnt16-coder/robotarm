using UnityEngine;

public class RobotCoordinateSystem : MonoBehaviour
{
    public Transform robotRoot;

    public Vector3 WorldToRobotLocalPosition(Vector3 worldPosition)
    {
        if (robotRoot == null)
        {
            Debug.LogError("Robot Root가 연결되지 않았습니다.");
            return Vector3.zero;
        }

        return robotRoot.InverseTransformPoint(worldPosition);
    }

    public Vector3 RobotLocalToWorldPosition(Vector3 localPosition)
    {
        if (robotRoot == null)
        {
            Debug.LogError("Robot Root가 연결되지 않았습니다.");
            return Vector3.zero;
        }

        return robotRoot.TransformPoint(localPosition);
    }

    public Vector3 WorldToRobotLocalDirection(Vector3 worldDirection)
    {
        if (robotRoot == null)
        {
            Debug.LogError("Robot Root가 연결되지 않았습니다.");
            return Vector3.zero;
        }

        return robotRoot.InverseTransformDirection(worldDirection);
    }

    public Quaternion WorldToRobotLocalRotation(Quaternion worldRotation)
    {
        if (robotRoot == null)
        {
            Debug.LogError("Robot Root가 연결되지 않았습니다.");
            return Quaternion.identity;
        }

        return Quaternion.Inverse(robotRoot.rotation) * worldRotation;
    }

    public void PrintBoxRobotCoordinate(Transform box)
    {
        if (box == null)
        {
            Debug.LogError("Box가 없습니다.");
            return;
        }

        Vector3 localPosition =
            WorldToRobotLocalPosition(box.position);

        Quaternion localRotation =
            WorldToRobotLocalRotation(box.rotation);

        Vector3 localForward =
            WorldToRobotLocalDirection(box.forward);

        Vector3 localUp =
            WorldToRobotLocalDirection(box.up);

        Vector3 localRight =
            WorldToRobotLocalDirection(box.right);

        Debug.Log("BOX ROBOT LOCAL POSITION = " + localPosition);

        Debug.Log("BOX ROBOT LOCAL ROTATION EULER = " + localRotation.eulerAngles);

        Debug.Log(
            "BOX ROBOT LOCAL FORWARD = " +
            localForward +
            " / UP = " +
            localUp +
            " / RIGHT = " +
            localRight
        );
    }
}