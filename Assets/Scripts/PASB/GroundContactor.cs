using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GroundContactor : MonoBehaviour
{
    public bool contact_ground = false;
    void OnCollisionEnter(Collision other) 
    {
        if (other.transform.tag == "ground")
        {
            contact_ground = true;
            //Debug.Log(transform.name + " contact the ground!");
        }
    }
    void OnCollisionExit(Collision other) 
    {
        if (other.transform.tag == "ground")
        {
            contact_ground = false;
            //Debug.Log(transform.name + " off the ground!");
        }
    }
}
