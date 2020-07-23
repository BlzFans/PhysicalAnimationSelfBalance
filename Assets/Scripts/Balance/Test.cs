using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{
    public Transform original;
    public Transform target;
    public ConfigurableJoint joint;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Quaternion delta_rotation = target.rotation * Quaternion.Inverse(original.rotation);
        delta_rotation.ToAngleAxis(out var angle, out var axis);
        axis = Vector3.ProjectOnPlane(axis, Vector3.up);
        Debug.Log(angle);
        Debug.DrawLine(original.position, original.position + axis, Color.red);
        Vector3 vel_direction = Quaternion.AngleAxis(-90f, Vector3.up) * axis;
        Debug.DrawLine(original.position, original.position + vel_direction, Color.blue);      
    }
}
