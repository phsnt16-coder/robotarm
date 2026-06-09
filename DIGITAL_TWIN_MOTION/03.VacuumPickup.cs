using UnityEngine;

public class VacuumPickup : MonoBehaviour
{
    public Transform gripCenter;

    public Transform targetBox;

    public Transform placeTarget;

    public BoxRandomSpawner boxRandomSpawner;

    public float placeTolerance = 10f;

    public bool spawnNextBoxOnPlaceSuccess = true;

    public float nextBoxSpawnDelay = 0.5f;

    private bool isHolding = false;

    private Vector3 heldLocalPosition = Vector3.zero;

    private Quaternion heldLocalRotation = Quaternion.identity;

    private Vector3 originalWorldScale;

    void Awake()
    {
        ResetRuntimeState();
    }

    void LateUpdate()
    {
        if (!isHolding || targetBox == null || gripCenter == null)
        {
            return;
        }

        targetBox.localPosition =
            heldLocalPosition;

        targetBox.localRotation =
            heldLocalRotation;

        Vector3 parentScale =
            gripCenter.lossyScale;

        targetBox.localScale =
            new Vector3(
                SafeDivide(originalWorldScale.x, parentScale.x),
                SafeDivide(originalWorldScale.y, parentScale.y),
                SafeDivide(originalWorldScale.z, parentScale.z)
            );
    }

    public void SetTargetBox(Transform newTargetBox)
    {
        targetBox =
            newTargetBox;

        Debug.Log("TARGET BOX SET: " + targetBox.name);
    }

    public void PickupBox()
    {
        if (isHolding || targetBox == null || gripCenter == null)
        {
            Debug.LogWarning("흡착 실패: TargetBox 또는 GripCenter 확인 필요");
            return;
        }

        Rigidbody rb =
            targetBox.GetComponent<Rigidbody>();

        if (rb != null)
        {
            rb.linearVelocity =
                Vector3.zero;

            rb.angularVelocity =
                Vector3.zero;

            rb.useGravity =
                false;

            rb.isKinematic =
                true;
        }

        originalWorldScale =
            targetBox.lossyScale;

        targetBox.SetParent(
            gripCenter,
            true
        );

        heldLocalPosition =
            Vector3.zero;

        heldLocalRotation =
            Quaternion.identity;

        targetBox.localPosition =
            heldLocalPosition;

        targetBox.localRotation =
            heldLocalRotation;

        isHolding =
            true;

        Debug.Log("VACUUM ON / BOX LOCKED TO GRIP_CENTER: " + targetBox.name);
    }

    public void ReleaseBox()
    {
        if (!isHolding || targetBox == null)
        {
            Debug.LogWarning("Release 실패: 잡고 있는 박스가 없습니다.");
            return;
        }

        Transform releasedBox =
            targetBox;

        Vector3 releasedWorldScale =
            releasedBox.lossyScale;

        releasedBox.SetParent(
            null,
            true
        );

        Vector3 finalPlacePosition =
            releasedBox.position;

        if (placeTarget != null)
        {
            float boxHeight =
                GetBoxHeight(releasedBox);

            finalPlacePosition =
                placeTarget.position +
                new Vector3(
                    0f,
                    boxHeight * 0.5f,
                    0f
                );

            releasedBox.position =
                finalPlacePosition;

            releasedBox.rotation =
                placeTarget.rotation;
        }

        releasedBox.localScale =
            releasedWorldScale;

        Rigidbody rb =
            releasedBox.GetComponent<Rigidbody>();

        if (rb != null)
        {
            rb.linearVelocity =
                Vector3.zero;

            rb.angularVelocity =
                Vector3.zero;

            rb.useGravity =
                true;

            rb.isKinematic =
                false;

            rb.constraints =
                RigidbodyConstraints.FreezeRotationX |
                RigidbodyConstraints.FreezeRotationZ;
        }

        bool placeSuccess =
            CheckPlaceSuccess(
                releasedBox,
                finalPlacePosition
            );

        if (placeSuccess)
        {
            Debug.Log("PLACE SUCCESS: " + releasedBox.name);

            if (spawnNextBoxOnPlaceSuccess)
            {
                Invoke(
                    nameof(SpawnNextBox),
                    nextBoxSpawnDelay
                );
            }
        }
        else
        {
            Debug.LogWarning("PLACE FAILED: " + releasedBox.name);
        }

        Debug.Log("VACUUM OFF / BOX RELEASED WITH GRAVITY: " + releasedBox.name);

        targetBox =
            null;

        isHolding =
            false;
    }

    public void DiscardHeldBox()
    {
        if (targetBox == null)
        {
            Debug.LogWarning("폐기할 박스가 없습니다.");

            isHolding =
                false;

            return;
        }

        Transform discardedBox =
            targetBox;

        discardedBox.SetParent(
            null,
            true
        );

        Debug.LogWarning("BOX DISCARDED: " + discardedBox.name);

        Destroy(
            discardedBox.gameObject
        );

        targetBox =
            null;

        isHolding =
            false;
    }

    void SpawnNextBox()
    {
        if (boxRandomSpawner == null)
        {
            Debug.LogWarning("BoxRandomSpawner가 연결되지 않았습니다.");
            return;
        }

        boxRandomSpawner.SpawnRandomBox();

        Debug.Log("NEXT BOX SPAWN REQUESTED");
    }

    float GetBoxHeight(Transform box)
    {
        Collider boxCollider =
            box.GetComponent<Collider>();

        if (boxCollider != null)
        {
            return boxCollider.bounds.size.y;
        }

        return box.lossyScale.y;
    }

    bool CheckPlaceSuccess(
        Transform box,
        Vector3 expectedPosition
    )
    {
        float distance =
            Vector3.Distance(
                box.position,
                expectedPosition
            );

        return distance <= placeTolerance;
    }

    float SafeDivide(
        float value,
        float divisor
    )
    {
        if (Mathf.Abs(divisor) < 0.0001f)
        {
            return value;
        }

        return value / divisor;
    }

    public bool IsHolding()
    {
        return isHolding;
    }

    [ContextMenu("Reset Runtime State")]
    public void ResetRuntimeState()
    {
        targetBox =
            null;

        isHolding =
            false;

        heldLocalPosition =
            Vector3.zero;

        heldLocalRotation =
            Quaternion.identity;
    }
}
