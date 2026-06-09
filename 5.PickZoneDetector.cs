using UnityEngine;

public class PickZoneDetector : MonoBehaviour
{
    public VacuumPickup vacuumPickup;

    private void OnTriggerEnter(Collider other)
    {
        ConveyorMove move = other.GetComponent<ConveyorMove>();

        if (move == null)
        {
            return;
        }

        move.StopMove();

        Rigidbody rb = other.GetComponent<Rigidbody>();

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.useGravity = false;
            rb.isKinematic = true;
        }

        if (vacuumPickup != null)
        {
            vacuumPickup.SetTargetBox(other.transform);
        }

        BoxArucoData arucoData =
            other.GetComponent<BoxArucoData>();

        if (arucoData != null)
        {
            arucoData.PrintArucoData();
        }
        else
        {
            Debug.LogWarning("BoxArucoData가 박스에 없습니다: " + other.name);
        }

        Debug.Log("BOX READY AT PICK_ZONE: " + other.name);
    }
}