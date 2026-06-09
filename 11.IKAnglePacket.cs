using UnityEngine;

[System.Serializable]
public class IKAnglePacket
{
    public float baseY;
    public float shoulderZ;
    public float elbowZ;
    public float gripperRotationZ;

    public IKAnglePacket(
        float baseY,
        float shoulderZ,
        float elbowZ,
        float gripperRotationZ
    )
    {
        this.baseY = baseY;
        this.shoulderZ = shoulderZ;
        this.elbowZ = elbowZ;
        this.gripperRotationZ = gripperRotationZ;
    }
}