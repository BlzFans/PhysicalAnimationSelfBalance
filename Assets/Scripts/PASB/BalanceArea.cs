using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BalanceArea : MonoBehaviour
{
    [HideInInspector]
    public BalanceFeet feet;
    [HideInInspector]
    public bool is_feet_in_area;
    [HideInInspector]
    public List<Collider> in_area_colliders = new List<Collider>();

    void OnTriggerEnter(Collider other) 
    {
        if (feet.feet_colliders.Contains(other) && !in_area_colliders.Contains(other))
            in_area_colliders.Add(other);
    }
    void OnTriggerExit(Collider other) 
    {
        if (feet.feet_colliders.Contains(other) && in_area_colliders.Contains(other))
            in_area_colliders.Remove(other);       
    }
    void FixedUpdate() 
    {
        // when feet collider overlap balance area we consider that feet is balanced
        is_feet_in_area = in_area_colliders.Count > 0;
    }

}
