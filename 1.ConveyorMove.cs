using UnityEngine;

public class ConveyorMove : MonoBehaviour
{
    public float speed = 50f;

    public bool isStopped = false;

    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        ResetRuntimeState();
    }

    void FixedUpdate()
    {
        if (isStopped)
        {
            StopPhysics();

            return;
        }

        if (rb != null)
        {
            Vector3 nextPosition =
                rb.position +
                Vector3.right * speed * Time.fixedDeltaTime;

            rb.MovePosition(nextPosition);
        }
        else
        {
            transform.Translate(
                Vector3.right * speed * Time.fixedDeltaTime,
                Space.World
            );
        }
    }

    public void StopMove()
    {
        isStopped = true;

        StopPhysics();

        Debug.Log("BOX STOPPED");
    }

    public void StartMove()
    {
        isStopped = false;

        if (rb != null)
        {
            rb.isKinematic = false;
            rb.useGravity = false;
        }
    }

    void StopPhysics()
    {
        if (rb == null)
        {
            return;
        }

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.useGravity = false;
        rb.isKinematic = true;
    }

    [ContextMenu("Reset Runtime State")]
    public void ResetRuntimeState()
    {
        isStopped = false;

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.useGravity = false;
            rb.isKinematic = false;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
        }
    }
}