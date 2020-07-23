using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Stabber : MonoBehaviour
{
    public float stab_speed = 3f;
    public float stab_distance = 3f;
    public float stab_wait_time = 1f;
    public float now_wait_time = 1f;
    float now_stab_sign = 1f;
    float now_stab_distance = 0f;
    Rigidbody rg;
    // Start is called before the first frame update
    void Start()
    {
        rg = GetComponentInChildren<Rigidbody>();
        rg.isKinematic = true;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (now_wait_time >= stab_wait_time)
        {
            Vector3 stab_move = now_stab_sign * transform.forward * stab_speed * Time.fixedDeltaTime;
            now_stab_distance += stab_move.magnitude;
            if (now_stab_distance >= stab_distance)
            {
                stab_move -= stab_move.normalized * (now_stab_distance - stab_distance);
                now_stab_sign *= -1f;
                now_stab_distance = 0f;
                now_wait_time = 0f;
            }
            rg.position += stab_move;
        }
        else
        {
            now_wait_time += Time.fixedDeltaTime;
        }        
    }
}
