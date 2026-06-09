using UnityEngine;

public class JointLimitProfile : MonoBehaviour
{
    public float ClampBase(float value)
    {
        return Mathf.Clamp(value, -180f, 180f);
    }

    public float ClampShoulder(float value)
    {
        return Mathf.Clamp(value, 0f, 180f);
    }

    public float ClampElbow(float value)
    {
        return Mathf.Clamp(value, -180f, 0f);
    }

    public float ClampGripperRotation(float value)
    {
        return Mathf.Clamp(value, -60f, 100f);
    }

    public float ClampGripper(float value)
    {
        return Mathf.Clamp(value, -90f, 90f);
    }

    public Vector3 ClampBasePose(Vector3 pose)
    {
        pose.y = ClampBase(pose.y);
        return pose;
    }

    public Vector3 ClampShoulderPose(Vector3 pose)
    {
        pose.z = ClampShoulder(pose.z);
        return pose;
    }

    public Vector3 ClampElbowPose(Vector3 pose)
    {
        pose.z = ClampElbow(pose.z);
        return pose;
    }

    public Vector3 ClampGripperRotationPose(Vector3 pose)
    {
        pose.z = ClampGripperRotation(pose.z);
        return pose;
    }

    public Vector3 ClampGripperPose(Vector3 pose)
    {
        pose.y = ClampGripper(pose.y);
        return pose;
    }
}
