using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Hitter : MonoBehaviour
{
    public float max_velocity_impulse = 15f;
    public float max_force_impulse = 15f;
    public float off_balance_force_gate = 3f;

    void OnCollisionEnter(Collision other) 
    {
        if (other.transform.root.tag == "character")
        {
            //Debug.Log(other.impulse.magnitude);
            AddOffBalanceImpulse(other.impulse, other.transform.root.GetComponentInChildren<ShowcaseAnimatorController>());  
        }
    }

    void AddOffBalanceImpulse(Vector3 impulse, ShowcaseAnimatorController sac)
    {
        if (impulse.magnitude > off_balance_force_gate)
        {
            sac.pab.in_self_balance_mode = true;
            float impulse_strength = Mathf.Lerp(0f, max_velocity_impulse, Mathf.InverseLerp(0f, max_force_impulse, impulse.magnitude));
            Vector3 force = Vector3.ProjectOnPlane(impulse.normalized * impulse_strength, Vector3.up);
            sac.rg.AddForce(force, ForceMode.VelocityChange);
        }
    }
}
