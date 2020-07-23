using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Sweeper : MonoBehaviour
{
    public float sweep_speed = 90f;
    public float sweep_wait_time = 1f;
    float now_max_val = 1f;
    float now_wait_time = 1f;
    float now_val = 0f;
    ConfigurableJoint joint;

    void Start()
    {
        joint = GetComponent<ConfigurableJoint>();
    }

    void FixedUpdate()
    {
        if (now_wait_time >= sweep_wait_time)
        {
            now_val += now_max_val * sweep_speed * Time.fixedDeltaTime;
            if (Mathf.Abs(now_val) >= 1f)
            {
                now_val = now_max_val;
                now_max_val *= -1f;
                now_wait_time = 0f;
            }
            joint.targetRotation = new Quaternion(0f, 0f, now_val, 1f);            
        }
        else
        {
            now_wait_time += Time.fixedDeltaTime;
        }

    }
}
