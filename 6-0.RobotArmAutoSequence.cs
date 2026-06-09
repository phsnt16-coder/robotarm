using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections;

public class RobotArmAutoSequence : MonoBehaviour
{
    [System.Serializable]
    public class BoxPickReference
    {
        public string boxLabel;
        public Vector3 basePose;
        public Vector3 shoulderPose;
        public Vector3 elbowPose;
        public Vector3 gripperRotationPose;
        public Vector3 gripperPose;
    }

    public Transform baseJoint;
    public Transform shoulderJoint;
    public Transform elbowJoint;
    public Transform gripperRotationJoint;
    public Transform gripperJoint;

    public VacuumPickup vacuumPickup;
    public JointLimitProfile jointLimitProfile;
    public UnityLiveBinTester unityLiveBinTester;

    private Quaternion baseHome;
    private Quaternion shoulderHome;
    private Quaternion elbowHome;
    private Quaternion gripperRotationHome;
    private Quaternion gripperHome;

    private IKAnglePacket pickIKPacket;
    private IKAnglePacket placeIKPacket;

    private bool pickIKReady = false;
    private bool placeIKReady = false;
    private bool isMoving = false;

    [Header("POSE 1 : Joint Check")]
    public Vector3 pose1CheckBase = new Vector3(0f, 10f, 0f);
    public Vector3 pose1CheckShoulder = new Vector3(0f, 0f, 10f);
    public Vector3 pose1CheckElbow = new Vector3(0f, 0f, -10f);
    public Vector3 pose1CheckGripperRotation = new Vector3(0f, 0f, 10f);
    public Vector3 pose1CheckGripper = new Vector3(0f, -10f, 0f);

    [Header("POSE 2 : Origin Z Pose")]
    public Vector3 pose2Base = Vector3.zero;
    public Vector3 pose2Shoulder = Vector3.zero;
    public Vector3 pose2Elbow = Vector3.zero;
    public Vector3 pose2GripperRotation = new Vector3(0f, 0f, 100f);
    public Vector3 pose2Gripper = Vector3.zero;

    [Header("POSE 3 : ACT_PICK_READY")]
    public Vector3 pose3Base = Vector3.zero;
    public Vector3 pose3Shoulder = new Vector3(0f, 0f, 60f);
    public Vector3 pose3Elbow = new Vector3(0f, 0f, -60f);
    public Vector3 pose3GripperRotation = Vector3.zero;
    public Vector3 pose3Gripper = Vector3.zero;

    [Header("POSE 4 : Box Pick Reference")]
    public bool useBoxPickReference = true;

    public BoxPickReference[] boxPickReferences =
    {
        new BoxPickReference
        {
            boxLabel = "A1",
            basePose = Vector3.zero,
            shoulderPose = new Vector3(0f, 0f, 90f),
            elbowPose = new Vector3(0f, 0f, -67f),
            gripperRotationPose = new Vector3(0f, 0f, -25f),
            gripperPose = Vector3.zero
        },
        new BoxPickReference
        {
            boxLabel = "B8",
            basePose = Vector3.zero,
            shoulderPose = new Vector3(0f, 0f, 90f),
            elbowPose = new Vector3(0f, 0f, -75f),
            gripperRotationPose = new Vector3(0f, 0f, -20f),
            gripperPose = Vector3.zero
        },
        new BoxPickReference
        {
            boxLabel = "B3",
            basePose = Vector3.zero,
            shoulderPose = new Vector3(0f, 0f, 95f),
            elbowPose = new Vector3(0f, 0f, -75f),
            gripperRotationPose = new Vector3(0f, 0f, -20f),
            gripperPose = Vector3.zero
        }
    };

    [Header("POSE 4 : Fallback")]
    public Vector3 pose4FallbackBase = Vector3.zero;
    public Vector3 pose4FallbackShoulder = new Vector3(0f, 0f, 90f);
    public Vector3 pose4FallbackElbow = new Vector3(0f, 0f, -67f);
    public Vector3 pose4FallbackGripperRotation = new Vector3(0f, 0f, -25f);
    public Vector3 pose4FallbackGripper = Vector3.zero;

    [Header("POSE 5 : SAFE_Z_POSE")]
    public Vector3 pose5Base = Vector3.zero;
    public Vector3 pose5Shoulder = Vector3.zero;
    public Vector3 pose5Elbow = Vector3.zero;
    public Vector3 pose5GripperRotation = new Vector3(0f, 0f, -25f);
    public Vector3 pose5Gripper = Vector3.zero;

    [Header("POSE 6 : GO_TO_PLACE")]
    public Vector3 pose6FallbackBase = new Vector3(0f, -90f, 0f);
    public Vector3 pose6FallbackShoulder = new Vector3(0f, 0f, 90f);
    public Vector3 pose6FallbackElbow = new Vector3(0f, 0f, -60f);
    public Vector3 pose6FallbackGripperRotation = new Vector3(0f, 0f, -25f);
    public Vector3 pose6FallbackGripper = Vector3.zero;

    [Header("Motion")]
    public float moveSpeed = 40f;
    public float poseDelay = 0.2f;
    public bool allowFallbackWhenNoIK = true;

    [Header("Auto Sequence")]
    public bool enableAutoSequenceKey = true;
    public float autoStepDelay = 0.25f;
    public bool deleteBoxWhenPlaceTargetFailed = true;
    public bool returnPose2WhenFailed = true;

    void Awake()
    {
        ResetRuntimeState();
    }

    void Start()
    {
        SaveHomePose();

        ResetRuntimeState();

        ForcePose2OriginOnPlay();
    }

    void OnDisable()
    {
        ResetRuntimeState();
    }

    void Update()
    {
        Keyboard keyboard = Keyboard.current;

        if (keyboard == null)
        {
            return;
        }

        if (keyboard.digit1Key.wasPressedThisFrame) StartPose1JointCheck();
        if (keyboard.digit2Key.wasPressedThisFrame) StartPose2OriginZ();
        if (keyboard.digit3Key.wasPressedThisFrame) StartPose3ActPickReady();
        if (keyboard.digit4Key.wasPressedThisFrame) StartPose4ActPickIK();
        if (keyboard.digit5Key.wasPressedThisFrame) StartPose5SafeZ();
        if (keyboard.digit6Key.wasPressedThisFrame) StartPose6GoToPlaceIK();
        if (keyboard.digit7Key.wasPressedThisFrame) StartPose7Release();

        if (keyboard.hKey.wasPressedThisFrame) StartPose2OriginZ();

        if (enableAutoSequenceKey && keyboard.aKey.wasPressedThisFrame)
        {
            StartAutoPickPlaceSequence();
        }
    }

    void SaveHomePose()
    {
        if (
            baseJoint == null ||
            shoulderJoint == null ||
            elbowJoint == null ||
            gripperRotationJoint == null ||
            gripperJoint == null
        )
        {
            Debug.LogError("RobotArmAutoSequence 조인트 연결을 확인하세요.");
            return;
        }

        baseHome = baseJoint.localRotation;
        shoulderHome = shoulderJoint.localRotation;
        elbowHome = elbowJoint.localRotation;
        gripperRotationHome = gripperRotationJoint.localRotation;
        gripperHome = gripperJoint.localRotation;

        Debug.Log("HOME ROTATION SAVED");
    }

    void ForcePose2OriginOnPlay()
    {
        MoveImmediately(
            pose2Base,
            pose2Shoulder,
            pose2Elbow,
            pose2GripperRotation,
            pose2Gripper
        );

        Debug.Log("PLAY START → POSE 2 ORIGIN APPLIED");
    }

    public void SetPickIKAngles(IKAnglePacket packet)
    {
        pickIKPacket = packet;
        pickIKReady = true;

        Debug.Log(
            "PICK IK ANGLES RECEIVED | Base: " +
            packet.baseY +
            ", Shoulder: " +
            packet.shoulderZ +
            ", Elbow: " +
            packet.elbowZ +
            ", GripperRot: " +
            packet.gripperRotationZ
        );
    }

    public void SetPlaceIKAngles(IKAnglePacket packet)
    {
        placeIKPacket = packet;
        placeIKReady = true;

        Debug.Log(
            "PLACE IK ANGLES RECEIVED | Base: " +
            packet.baseY +
            ", Shoulder: " +
            packet.shoulderZ +
            ", Elbow: " +
            packet.elbowZ +
            ", GripperRot: " +
            packet.gripperRotationZ
        );
    }

    public void StartAutoPickPlaceSequence()
    {
        if (!isMoving)
        {
            StartCoroutine(AutoPickPlaceRoutine());
        }
    }

    IEnumerator AutoPickPlaceRoutine()
    {
        isMoving = true;

        Debug.Log("AUTO SEQUENCE START : POSE 3 → 4 → 5 → 6 → 7");

        yield return StartCoroutine(
            MoveToPoseInternal(
                pose3Base,
                pose3Shoulder,
                pose3Elbow,
                pose3GripperRotation,
                pose3Gripper
            )
        );

        Debug.Log("AUTO POSE 3 : ACT PICK READY");

        yield return new WaitForSeconds(autoStepDelay);

        bool pickSuccess =
            false;

        yield return StartCoroutine(
            Pose4PickInternal(
                result =>
                {
                    pickSuccess = result;
                }
            )
        );

        if (!pickSuccess)
        {
            Debug.LogWarning("AUTO SEQUENCE FAILED : POSE 4 PICK 실패");

            yield return StartCoroutine(
                HandlePlaceTargetFailureRoutine()
            );

            isMoving = false;

            yield break;
        }

        yield return new WaitForSeconds(autoStepDelay);

        yield return StartCoroutine(
            MoveToPoseInternal(
                pose5Base,
                pose5Shoulder,
                pose5Elbow,
                pose5GripperRotation,
                pose5Gripper
            )
        );

        Debug.Log("AUTO POSE 5 : SAFE Z");

        yield return new WaitForSeconds(autoStepDelay);

        bool placeMoveSuccess =
            false;

        yield return StartCoroutine(
            Pose6PlaceInternal(
                result =>
                {
                    placeMoveSuccess = result;
                }
            )
        );

        if (!placeMoveSuccess)
        {
            Debug.LogWarning("AUTO SEQUENCE FAILED : 적재 좌표 생성 또는 이동 실패");

            yield return StartCoroutine(
                HandlePlaceTargetFailureRoutine()
            );

            isMoving = false;

            yield break;
        }

        yield return new WaitForSeconds(autoStepDelay);

        if (vacuumPickup != null)
        {
            vacuumPickup.ReleaseBox();
        }

        Debug.Log("AUTO POSE 7 : RELEASE DONE");

        isMoving = false;
    }

    IEnumerator HandlePlaceTargetFailureRoutine()
    {
        if (
            deleteBoxWhenPlaceTargetFailed &&
            vacuumPickup != null &&
            vacuumPickup.targetBox != null
        )
        {
            Transform failedBox =
                vacuumPickup.targetBox;

            failedBox.SetParent(
                null,
                true
            );

            Destroy(
                failedBox.gameObject
            );

            vacuumPickup.ResetRuntimeState();

            Debug.LogWarning("PLACE FAILED → BOX DELETED");
        }

        if (returnPose2WhenFailed)
        {
            yield return new WaitForSeconds(poseDelay);

            yield return StartCoroutine(
                MoveToPoseInternal(
                    pose2Base,
                    pose2Shoulder,
                    pose2Elbow,
                    pose2GripperRotation,
                    pose2Gripper
                )
            );

            Debug.Log("PLACE FAILED → RETURN TO POSE 2");
        }
    }

    public void StartPose1JointCheck()
    {
        if (!isMoving)
        {
            StartCoroutine(Pose1JointCheckRoutine());
        }
    }

    IEnumerator Pose1JointCheckRoutine()
    {
        isMoving = true;

        yield return StartCoroutine(
            MoveToPoseInternal(
                pose1CheckBase,
                pose1CheckShoulder,
                pose1CheckElbow,
                pose1CheckGripperRotation,
                pose1CheckGripper
            )
        );

        yield return new WaitForSeconds(poseDelay);

        yield return StartCoroutine(
            MoveToPoseInternal(
                pose2Base,
                pose2Shoulder,
                pose2Elbow,
                pose2GripperRotation,
                pose2Gripper
            )
        );

        Debug.Log("POSE 1 : JOINT CHECK DONE");

        isMoving = false;
    }

    public void StartPose2OriginZ()
    {
        if (!isMoving)
        {
            StartCoroutine(
                MoveToPose(
                    pose2Base,
                    pose2Shoulder,
                    pose2Elbow,
                    pose2GripperRotation,
                    pose2Gripper,
                    "POSE 2 : ORIGIN Z"
                )
            );
        }
    }

    public void StartPose3ActPickReady()
    {
        if (!isMoving)
        {
            StartCoroutine(
                MoveToPose(
                    pose3Base,
                    pose3Shoulder,
                    pose3Elbow,
                    pose3GripperRotation,
                    pose3Gripper,
                    "POSE 3 : ACT PICK READY"
                )
            );
        }
    }

    public void StartPose4ActPickIK()
    {
        if (!isMoving)
        {
            StartCoroutine(Pose4PickIKRoutine());
        }
    }

    IEnumerator Pose4PickIKRoutine()
    {
        isMoving = true;

        bool result =
            false;

        yield return StartCoroutine(
            Pose4PickInternal(
                value =>
                {
                    result = value;
                }
            )
        );

        isMoving = false;
    }

    IEnumerator Pose4PickInternal(System.Action<bool> onComplete)
    {
        if (pickIKReady)
        {
            yield return StartCoroutine(
                MoveToIKPacketInternal(pickIKPacket)
            );
        }
        else
        {
            if (!allowFallbackWhenNoIK)
            {
                Debug.LogWarning("POSE 4 실행 불가: Pick IK 값이 없습니다.");

                onComplete(false);

                yield break;
            }

            Vector3 basePose;
            Vector3 shoulderPose;
            Vector3 elbowPose;
            Vector3 gripperRotationPose;
            Vector3 gripperPose;

            GetBoxPickReferencePose(
                out basePose,
                out shoulderPose,
                out elbowPose,
                out gripperRotationPose,
                out gripperPose
            );

            yield return StartCoroutine(
                MoveToPoseInternal(
                    basePose,
                    shoulderPose,
                    elbowPose,
                    gripperRotationPose,
                    gripperPose
                )
            );
        }

        yield return new WaitForSeconds(poseDelay);

        if (vacuumPickup != null)
        {
            vacuumPickup.PickupBox();
        }

        Debug.Log("POSE 4 : ACT PICK DONE");

        onComplete(true);
    }

    void GetBoxPickReferencePose(
        out Vector3 basePose,
        out Vector3 shoulderPose,
        out Vector3 elbowPose,
        out Vector3 gripperRotationPose,
        out Vector3 gripperPose
    )
    {
        basePose = pose4FallbackBase;
        shoulderPose = pose4FallbackShoulder;
        elbowPose = pose4FallbackElbow;
        gripperRotationPose = pose4FallbackGripperRotation;
        gripperPose = pose4FallbackGripper;

        if (
            !useBoxPickReference ||
            vacuumPickup == null ||
            vacuumPickup.targetBox == null
        )
        {
            Debug.LogWarning("Box Reference 없음 → Pose4 Fallback 사용");
            return;
        }

        string label = GetCurrentBoxLabel();

        for (int i = 0; i < boxPickReferences.Length; i++)
        {
            if (boxPickReferences[i].boxLabel == label)
            {
                basePose = boxPickReferences[i].basePose;
                shoulderPose = boxPickReferences[i].shoulderPose;
                elbowPose = boxPickReferences[i].elbowPose;
                gripperRotationPose = boxPickReferences[i].gripperRotationPose;
                gripperPose = boxPickReferences[i].gripperPose;

                Debug.Log("POSE 4 BOX REFERENCE USED: " + label);

                return;
            }
        }

        Debug.LogWarning("해당 박스 Reference 없음 → Pose4 Fallback 사용: " + label);
    }

    string GetCurrentBoxLabel()
    {
        if (vacuumPickup == null || vacuumPickup.targetBox == null)
        {
            return "UNKNOWN";
        }

        BoxArucoData arucoData =
            vacuumPickup.targetBox.GetComponent<BoxArucoData>();

        if (
            arucoData != null &&
            !string.IsNullOrEmpty(arucoData.boxLabel)
        )
        {
            return arucoData.boxLabel;
        }

        string objectName =
            vacuumPickup.targetBox.name;

        if (objectName.Contains("A1")) return "A1";
        if (objectName.Contains("B3")) return "B3";
        if (objectName.Contains("B8")) return "B8";

        return "UNKNOWN";
    }

    public void StartPose5SafeZ()
    {
        if (!isMoving)
        {
            StartCoroutine(
                MoveToPose(
                    pose5Base,
                    pose5Shoulder,
                    pose5Elbow,
                    pose5GripperRotation,
                    pose5Gripper,
                    "POSE 5 : SAFE Z"
                )
            );
        }
    }

    public void StartPose6GoToPlaceIK()
    {
        if (!isMoving)
        {
            StartCoroutine(Pose6PlaceIKRoutine());
        }
    }

    IEnumerator Pose6PlaceIKRoutine()
    {
        isMoving = true;

        bool result =
            false;

        yield return StartCoroutine(
            Pose6PlaceInternal(
                value =>
                {
                    result = value;
                }
            )
        );

        if (!result)
        {
            yield return StartCoroutine(
                HandlePlaceTargetFailureRoutine()
            );
        }

        isMoving = false;
    }

    IEnumerator Pose6PlaceInternal(System.Action<bool> onComplete)
    {
        float binPackingRotationAngle =
            0f;

        bool prepareSuccess =
            PrepareAlgorithmPlaceTarget(
                out binPackingRotationAngle
            );

        if (!prepareSuccess)
        {
            onComplete(false);

            yield break;
        }

        if (placeIKReady)
        {
            yield return StartCoroutine(
                MoveToIKPacketInternalWithPlaceRotation(
                    placeIKPacket,
                    binPackingRotationAngle
                )
            );
        }
        else
        {
            if (!allowFallbackWhenNoIK)
            {
                Debug.LogWarning("POSE 6 실행 불가: Place IK 값이 없습니다.");

                onComplete(false);

                yield break;
            }

            Vector3 finalGripperPose =
                pose6FallbackGripper +
                new Vector3(
                    0f,
                    binPackingRotationAngle,
                    0f
                );

            yield return StartCoroutine(
                MoveToPoseInternal(
                    pose6FallbackBase,
                    pose6FallbackShoulder,
                    pose6FallbackElbow,
                    pose6FallbackGripperRotation,
                    finalGripperPose
                )
            );
        }

        Debug.Log("POSE 6 : GO TO PLACE DONE");

        onComplete(true);
    }

    bool PrepareAlgorithmPlaceTarget(
        out float binPackingRotationAngle
    )
    {
        binPackingRotationAngle =
            0f;

        if (unityLiveBinTester == null)
        {
            Debug.LogWarning("Bin Packing 준비 실패: UnityLiveBinTester 연결 없음");
            return false;
        }

        if (vacuumPickup == null)
        {
            Debug.LogWarning("Bin Packing 준비 실패: VacuumPickup 연결 없음");
            return false;
        }

        if (vacuumPickup.targetBox == null)
        {
            Debug.LogWarning("Bin Packing 준비 실패: TargetBox 없음");
            return false;
        }

        bool success =
            unityLiveBinTester.PreparePlaceTargetForBox(
                vacuumPickup.targetBox
            );

        if (!success)
        {
            Debug.LogWarning("BIN PACKING TARGET PREPARE FAILED");
            return false;
        }

        binPackingRotationAngle =
            unityLiveBinTester.currentPlaceRotationAngle;

        Debug.Log(
            "BIN PACKING TARGET PREPARED | Rotation: " +
            binPackingRotationAngle
        );

        return true;
    }

    public void StartPose7Release()
    {
        if (!isMoving)
        {
            StartCoroutine(Pose7ReleaseRoutine());
        }
    }

    IEnumerator Pose7ReleaseRoutine()
    {
        isMoving = true;

        if (vacuumPickup != null)
        {
            vacuumPickup.ReleaseBox();
        }

        yield return new WaitForSeconds(poseDelay);

        Debug.Log("POSE 7 : RELEASE DONE");

        isMoving = false;
    }

    IEnumerator MoveToIKPacketInternal(IKAnglePacket packet)
    {
        Vector3 basePose =
            new Vector3(0f, packet.baseY, 0f);

        Vector3 shoulderPose =
            new Vector3(0f, 0f, packet.shoulderZ);

        Vector3 elbowPose =
            new Vector3(0f, 0f, packet.elbowZ);

        Vector3 gripperRotationPose =
            new Vector3(0f, 0f, packet.gripperRotationZ);

        Vector3 gripperPose =
            pose2Gripper;

        yield return StartCoroutine(
            MoveToPoseInternal(
                basePose,
                shoulderPose,
                elbowPose,
                gripperRotationPose,
                gripperPose
            )
        );
    }

    IEnumerator MoveToIKPacketInternalWithPlaceRotation(
        IKAnglePacket packet,
        float binPackingRotationAngle
    )
    {
        Vector3 basePose =
            new Vector3(0f, packet.baseY, 0f);

        Vector3 shoulderPose =
            new Vector3(0f, 0f, packet.shoulderZ);

        Vector3 elbowPose =
            new Vector3(0f, 0f, packet.elbowZ);

        Vector3 gripperRotationPose =
            new Vector3(0f, 0f, packet.gripperRotationZ);

        Vector3 gripperPose =
            pose2Gripper +
            new Vector3(
                0f,
                binPackingRotationAngle,
                0f
            );

        yield return StartCoroutine(
            MoveToPoseInternal(
                basePose,
                shoulderPose,
                elbowPose,
                gripperRotationPose,
                gripperPose
            )
        );
    }

    IEnumerator MoveToPose(
        Vector3 baseOffset,
        Vector3 shoulderOffset,
        Vector3 elbowOffset,
        Vector3 gripperRotationOffset,
        Vector3 gripperOffset,
        string arriveMessage
    )
    {
        isMoving = true;

        yield return StartCoroutine(
            MoveToPoseInternal(
                baseOffset,
                shoulderOffset,
                elbowOffset,
                gripperRotationOffset,
                gripperOffset
            )
        );

        Debug.Log(arriveMessage);

        isMoving = false;
    }

    IEnumerator MoveToPoseInternal(
        Vector3 baseOffset,
        Vector3 shoulderOffset,
        Vector3 elbowOffset,
        Vector3 gripperRotationOffset,
        Vector3 gripperOffset
    )
    {
        Vector3 limitedBase = baseOffset;
        Vector3 limitedShoulder = shoulderOffset;
        Vector3 limitedElbow = elbowOffset;
        Vector3 limitedGripperRotation = gripperRotationOffset;
        Vector3 limitedGripper = gripperOffset;

        ApplyJointLimits(
            ref limitedBase,
            ref limitedShoulder,
            ref limitedElbow,
            ref limitedGripperRotation,
            ref limitedGripper
        );

        Quaternion baseTarget =
            baseHome * Quaternion.Euler(limitedBase);

        Quaternion shoulderTarget =
            shoulderHome * Quaternion.Euler(limitedShoulder);

        Quaternion elbowTarget =
            elbowHome * Quaternion.Euler(limitedElbow);

        Quaternion gripperRotationTarget =
            gripperRotationHome * Quaternion.Euler(limitedGripperRotation);

        Quaternion gripperTarget =
            gripperHome * Quaternion.Euler(limitedGripper);

        while (
            Quaternion.Angle(baseJoint.localRotation, baseTarget) > 0.5f ||
            Quaternion.Angle(shoulderJoint.localRotation, shoulderTarget) > 0.5f ||
            Quaternion.Angle(elbowJoint.localRotation, elbowTarget) > 0.5f ||
            Quaternion.Angle(gripperRotationJoint.localRotation, gripperRotationTarget) > 0.5f ||
            Quaternion.Angle(gripperJoint.localRotation, gripperTarget) > 0.5f
        )
        {
            baseJoint.localRotation =
                Quaternion.RotateTowards(
                    baseJoint.localRotation,
                    baseTarget,
                    moveSpeed * Time.deltaTime
                );

            shoulderJoint.localRotation =
                Quaternion.RotateTowards(
                    shoulderJoint.localRotation,
                    shoulderTarget,
                    moveSpeed * Time.deltaTime
                );

            elbowJoint.localRotation =
                Quaternion.RotateTowards(
                    elbowJoint.localRotation,
                    elbowTarget,
                    moveSpeed * Time.deltaTime
                );

            gripperRotationJoint.localRotation =
                Quaternion.RotateTowards(
                    gripperRotationJoint.localRotation,
                    gripperRotationTarget,
                    moveSpeed * Time.deltaTime
                );

            gripperJoint.localRotation =
                Quaternion.RotateTowards(
                    gripperJoint.localRotation,
                    gripperTarget,
                    moveSpeed * Time.deltaTime
                );

            yield return null;
        }

        baseJoint.localRotation = baseTarget;
        shoulderJoint.localRotation = shoulderTarget;
        elbowJoint.localRotation = elbowTarget;
        gripperRotationJoint.localRotation = gripperRotationTarget;
        gripperJoint.localRotation = gripperTarget;
    }

    void MoveImmediately(
        Vector3 baseOffset,
        Vector3 shoulderOffset,
        Vector3 elbowOffset,
        Vector3 gripperRotationOffset,
        Vector3 gripperOffset
    )
    {
        ApplyJointLimits(
            ref baseOffset,
            ref shoulderOffset,
            ref elbowOffset,
            ref gripperRotationOffset,
            ref gripperOffset
        );

        baseJoint.localRotation =
            baseHome * Quaternion.Euler(baseOffset);

        shoulderJoint.localRotation =
            shoulderHome * Quaternion.Euler(shoulderOffset);

        elbowJoint.localRotation =
            elbowHome * Quaternion.Euler(elbowOffset);

        gripperRotationJoint.localRotation =
            gripperRotationHome * Quaternion.Euler(gripperRotationOffset);

        gripperJoint.localRotation =
            gripperHome * Quaternion.Euler(gripperOffset);
    }

    void ApplyJointLimits(
        ref Vector3 basePose,
        ref Vector3 shoulderPose,
        ref Vector3 elbowPose,
        ref Vector3 gripperRotationPose,
        ref Vector3 gripperPose
    )
    {
        if (jointLimitProfile == null)
        {
            return;
        }

        basePose =
            jointLimitProfile.ClampBasePose(basePose);

        shoulderPose =
            jointLimitProfile.ClampShoulderPose(shoulderPose);

        elbowPose =
            jointLimitProfile.ClampElbowPose(elbowPose);

        gripperRotationPose =
            jointLimitProfile.ClampGripperRotationPose(gripperRotationPose);

        gripperPose =
            jointLimitProfile.ClampGripperPose(gripperPose);
    }

    [ContextMenu("Reset Runtime State")]
    public void ResetRuntimeState()
    {
        isMoving = false;

        pickIKReady = false;

        placeIKReady = false;

        StopAllCoroutines();
    }
}