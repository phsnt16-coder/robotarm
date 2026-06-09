using UnityEngine;

public class BoxArucoData : MonoBehaviour
{
    public int markerId;

    public string boxLabel;

    public float targetAngle;

    public int upAxisIdx;

    [Header("Aruco Marker")]
    public Transform arucoMarker;

    [Header("Marker To Box Center Offset - mm")]
    public Vector3 markerToBoxCenterOffsetMm = Vector3.zero;

    [Header("Output")]
    public Vector3 markerCoordsMm;

    public Vector3 boxCenterCoordsMm;

    public void UpdateCoordsFromAruco()
    {
        if (arucoMarker == null)
        {
            Debug.LogWarning("Aruco Marker가 연결되지 않았습니다: " + gameObject.name);

            markerCoordsMm = transform.position;

            boxCenterCoordsMm = transform.position;

            return;
        }

        markerCoordsMm =
            arucoMarker.position;

        boxCenterCoordsMm =
            markerCoordsMm + markerToBoxCenterOffsetMm;
    }

    public void PrintArucoData()
    {
        UpdateCoordsFromAruco();

        Debug.Log(
            "ARUCO BOX DATA | " +
            "Label: " + boxLabel +
            ", MarkerID: " + markerId +
            ", TargetAngle: " + targetAngle.ToString("F2") +
            ", MarkerCoordsMM: " + markerCoordsMm.ToString("F2") +
            ", BoxCenterCoordsMM: " + boxCenterCoordsMm.ToString("F2") +
            ", UpAxisIdx: " + upAxisIdx
        );
    }
}