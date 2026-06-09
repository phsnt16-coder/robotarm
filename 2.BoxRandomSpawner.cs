using UnityEngine;
using UnityEngine.InputSystem;

public class BoxRandomSpawner : MonoBehaviour
{
    public Transform spawnPoint;

    public GameObject a1Prefab;
    public GameObject b3Prefab;
    public GameObject b8Prefab;

    public float beltTopY = 66f;

    public bool spawnOnStart = true;

    public int maxTotalCount = 6;

    public int maxEachTypeCount = 2;

    private int spawnCount = 0;

    private int a1Count = 0;

    private int b3Count = 0;

    private int b8Count = 0;

    void Start()
    {
        if (spawnOnStart)
        {
            SpawnRandomBox();
        }
    }

    void Update()
    {
        Keyboard keyboard = Keyboard.current;

        if (keyboard == null)
        {
            return;
        }

        if (keyboard.nKey.wasPressedThisFrame)
        {
            SpawnRandomBox();
        }
    }

    public void SpawnRandomBox()
    {
        if (spawnCount >= maxTotalCount)
        {
            Debug.LogWarning("박스 최대 생성 개수에 도달했습니다.");
            return;
        }

        GameObject selectedPrefab =
            GetRandomAvailablePrefab();

        if (spawnPoint == null || selectedPrefab == null)
        {
            Debug.LogWarning("생성 가능한 박스가 없거나 SpawnPoint 연결이 없습니다.");
            return;
        }

        Vector3 spawnPos =
            spawnPoint.position;

        Vector3 boxSize =
            selectedPrefab.transform.localScale;

        spawnPos.y =
            beltTopY + (boxSize.y / 2f);

        GameObject box =
            Instantiate(
                selectedPrefab,
                spawnPos,
                spawnPoint.rotation
            );

        box.name =
            selectedPrefab.name +
            "_" +
            spawnCount;

        spawnCount++;

        IncreaseTypeCount(selectedPrefab);

        Rigidbody rb =
            box.GetComponent<Rigidbody>();

        if (rb != null)
        {
            rb.useGravity = false;
            rb.isKinematic = true;
        }

        ConveyorMove move =
            box.GetComponent<ConveyorMove>();

        if (move != null)
        {
            move.enabled = true;
            move.isStopped = false;
        }
        else
        {
            Debug.LogError("생성된 박스에 ConveyorMove가 없습니다: " + box.name);
        }

        Debug.Log(
            "BOX SPAWNED: " +
            box.name +
            " | A1: " +
            a1Count +
            " / B3: " +
            b3Count +
            " / B8: " +
            b8Count +
            " / Total: " +
            spawnCount
        );
    }

    GameObject GetRandomAvailablePrefab()
    {
        GameObject[] candidates =
            new GameObject[3];

        int candidateCount = 0;

        if (a1Prefab != null && a1Count < maxEachTypeCount)
        {
            candidates[candidateCount] = a1Prefab;
            candidateCount++;
        }

        if (b3Prefab != null && b3Count < maxEachTypeCount)
        {
            candidates[candidateCount] = b3Prefab;
            candidateCount++;
        }

        if (b8Prefab != null && b8Count < maxEachTypeCount)
        {
            candidates[candidateCount] = b8Prefab;
            candidateCount++;
        }

        if (candidateCount == 0)
        {
            return null;
        }

        int index =
            Random.Range(
                0,
                candidateCount
            );

        return candidates[index];
    }

    void IncreaseTypeCount(GameObject selectedPrefab)
    {
        if (selectedPrefab == a1Prefab)
        {
            a1Count++;
            return;
        }

        if (selectedPrefab == b3Prefab)
        {
            b3Count++;
            return;
        }

        if (selectedPrefab == b8Prefab)
        {
            b8Count++;
        }
    }

    [ContextMenu("Reset Spawn Count")]
    public void ResetSpawnCount()
    {
        spawnCount = 0;

        a1Count = 0;

        b3Count = 0;

        b8Count = 0;

        Debug.Log("SPAWN COUNT RESET");
    }
}