using UnityEngine;

public class PlaceSlotManager : MonoBehaviour
{
    public Transform placeTarget;

    public Transform[] placeSlots;

    private int currentSlotIndex = 0;

    public bool TryMovePlaceTargetToNextSlot()
    {
        if (placeTarget == null)
        {
            Debug.LogError("PlaceTarget이 연결되지 않았습니다.");
            return false;
        }

        if (placeSlots == null || placeSlots.Length == 0)
        {
            Debug.LogError("PlaceSlot이 없습니다.");
            return false;
        }

        if (currentSlotIndex >= placeSlots.Length)
        {
            Debug.LogWarning("더 이상 적재 가능한 슬롯이 없습니다.");
            return false;
        }

        Transform targetSlot = placeSlots[currentSlotIndex];

        if (targetSlot == null)
        {
            Debug.LogError("현재 슬롯이 비어 있습니다: " + currentSlotIndex);
            return false;
        }

        placeTarget.position = targetSlot.position;
        placeTarget.rotation = targetSlot.rotation;

        Debug.Log("PLACE TARGET MOVED TO SLOT_" + currentSlotIndex);

        currentSlotIndex++;

        return true;
    }

    [ContextMenu("Reset Slot Index")]
    public void ResetSlotIndex()
    {
        currentSlotIndex = 0;

        Debug.Log("PLACE SLOT INDEX RESET");
    }
}