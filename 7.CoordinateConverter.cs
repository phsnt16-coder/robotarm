using UnityEngine;

public class CoordinateConverter : MonoBehaviour
{
    public Transform placeOrigin;

    public Vector3 ConvertPythonBoxToUnityCenter(
        float x,
        float y,
        float z,
        float w,
        float h,
        float d
    )
    {
        float unityX = placeOrigin.position.x + x + (w / 2f);

        float unityY = placeOrigin.position.y + z + (h / 2f);

        float unityZ = placeOrigin.position.z + y + (d / 2f);

        return new Vector3(unityX, unityY, unityZ);
    }

    public Vector3 ConvertPythonLoadCoords(
        float loadX,
        float loadY,
        float loadZ
    )
    {
        float unityX = placeOrigin.position.x + loadX;

        float unityY = placeOrigin.position.y + loadZ;

        float unityZ = placeOrigin.position.z + loadY;

        return new Vector3(unityX, unityY, unityZ);
    }
}