using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestFeet : MonoBehaviour
{
    public Transform target;
    public Transform bias_target;
    public Quaternion bias_rotation;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log(Vector3.zero.normalized);
        bias_rotation = Quaternion.Inverse(target.rotation) * bias_target.rotation;

    }

    // Update is called once per frame
    void Update()
    {
        target.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(transform.forward, Vector3.up), Vector3.up);
        bias_target.rotation = target.rotation * bias_rotation;

        //Quaternion delta_rotation = transform.rotation * Quaternion.Inverse(target.rotation);
        //delta_rotation.ToAngleAxis(out var angle, out var axis);
        
        Vector3 axis = CalculateEulerAngleDiff(bias_target.rotation, transform.rotation);
        axis = Vector3.ProjectOnPlane(axis, Vector3.up);
        Vector3 rot_vel_direction = (Quaternion.AngleAxis(-90f, Vector3.up) * axis).normalized;
        float angle = Quaternion.Angle(bias_target.rotation, transform.rotation);

        Debug.Log(angle);
        Debug.DrawLine(transform.position, transform.position + rot_vel_direction * 10f, Color.yellow);        
    }

    Vector3 CalculateEulerAngleDiff(Quaternion last_rotation, Quaternion now_rotation)
    {
        Quaternion delta_rotation = now_rotation * Quaternion.Inverse(last_rotation);
        Vector3 euler_rotation = new Vector3(
            Mathf.DeltaAngle(0, delta_rotation.eulerAngles.x),
            Mathf.DeltaAngle(0, delta_rotation.eulerAngles.y),
            Mathf.DeltaAngle(0, delta_rotation.eulerAngles.z));
        return euler_rotation;
    }

}
