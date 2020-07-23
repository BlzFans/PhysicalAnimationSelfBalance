using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MonsterBalanceFeet : MonoBehaviour
{
    public float feet_rotate_speed = 90f;
    public float garound_height = 0f;
    const float max_ray_length = 1f;
    float constant_feet_height;
    Vector3 garound_normal;
    Vector3 joint_init_x_axis;
    Vector3 joint_init_y_axis;
    Collider feet_collider;
    ConfigurableJoint feet_joint;
    // Start is called before the first frame update
    void Start()
    {
        feet_joint = GetComponent<ConfigurableJoint>();
        constant_feet_height = feet_joint.connectedBody.transform.TransformPoint(feet_joint.connectedAnchor).y - garound_height;
        garound_normal = Vector3.up;
        joint_init_x_axis = feet_joint.connectedBody.transform.InverseTransformDirection(feet_joint.transform.right);
        joint_init_y_axis = feet_joint.connectedBody.transform.InverseTransformDirection(feet_joint.transform.up);
        feet_collider = GetComponent<Collider>();
    }
    private void FixedUpdate() {
        // Debug
        //FeetFixedUpdate();
    }

    // Update is called once per frame
    public void FeetFixedUpdate()
    {
        RaycastHit[] allHits;
        allHits = Physics.RaycastAll(transform.position, -Vector3.up, max_ray_length);
        foreach (var hit in allHits)
        {
            if (hit.collider != feet_collider)
            {
                garound_height = hit.point.y + constant_feet_height;
                garound_normal = hit.normal;
                break;                
            }
        }
        Vector3 x_axis = feet_joint.connectedBody.transform.TransformDirection(joint_init_x_axis);
        Vector3 y_axis = feet_joint.connectedBody.transform.TransformDirection(joint_init_y_axis);        
        Vector3 z_axis = Vector3.Cross(x_axis, y_axis);
        Quaternion axis_rotation = Quaternion.LookRotation(z_axis, y_axis);

        Vector3 forward_axis = Vector3.Cross(transform.right, garound_normal);
        Quaternion target_rotation = Quaternion.LookRotation(forward_axis, garound_normal);

        Quaternion now_target_rotation = Quaternion.RotateTowards(transform.rotation, target_rotation, feet_rotate_speed * Time.fixedDeltaTime);
        Quaternion feet_local_rotation = Quaternion.Inverse(now_target_rotation) * axis_rotation;
        
        feet_joint.targetRotation = feet_local_rotation;
    }
}
