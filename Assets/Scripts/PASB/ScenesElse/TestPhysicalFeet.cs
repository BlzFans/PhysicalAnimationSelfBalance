using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestPhysicalFeet : MonoBehaviour
{
    public Transform target;
    public Transform other_leg;
    public float leg_avoid_length = 0.1f;
    public float avoid_move_scaler = 2f;
    public float back_speed = 1f;
    public float move_speed = 2f;
    public float position_param = 100f;
    public float velocity_param = 10f;
    Rigidbody rg;
    Collider col;
    Vector3 start_position;
    Vector3 now_target_position;
    Vector3 feet_vertical_move;
    // Start is called before the first frame update
    void Start()
    {
        rg = GetComponent<Rigidbody>();
        col = GetComponent<Collider>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (Input.GetKeyDown("w"))
        {
            start_position = rg.position;
            now_target_position = rg.position;
            feet_vertical_move = Vector3.zero;
        }
        if (Input.GetKey("w"))
        {
            Vector3 target_dir = target.position - start_position;
            target_dir = Vector3.ProjectOnPlane(target_dir, Vector3.up);
            Vector3 now_add_move = target_dir.normalized * (move_speed * Time.fixedDeltaTime);
            now_target_position += now_add_move;

            RaycastHit[] hits = rg.SweepTestAll(now_target_position - rg.position, leg_avoid_length);
            RaycastHit closest_hit = new RaycastHit();
            closest_hit.distance = Mathf.Infinity;
            foreach (var hit in hits)
            {
                if (hit.transform.tag == "leg")
                {
                    if (hit.distance < closest_hit.distance)
                        closest_hit = hit;
                }
            }
            if (closest_hit.distance != Mathf.Infinity)
            {
                Vector3 inward_direction = other_leg.position - closest_hit.point;
                inward_direction = Vector3.ProjectOnPlane(inward_direction, Vector3.up);

                float inward_angle = Vector3.SignedAngle(inward_direction, target_dir, Vector3.up);
                float abs_inward_angle = Mathf.Abs(inward_angle);
                if (abs_inward_angle > 90f) abs_inward_angle = 180f - abs_inward_angle;
                float angle_sign = inward_angle < 0f ? -1f : 1f;
                float tangent_angle = Mathf.Abs(90f - abs_inward_angle);
                Vector3 vertical_dir = Quaternion.AngleAxis(angle_sign * 90f, Vector3.up) * target_dir.normalized;
                float feet_vertical_move_length = now_add_move.magnitude * Mathf.Tan(Mathf.Abs(tangent_angle) * Mathf.Deg2Rad);
                float errot_correct_move_length = Mathf.Clamp(leg_avoid_length - closest_hit.distance, 0f, leg_avoid_length);
                feet_vertical_move = vertical_dir * (feet_vertical_move_length + errot_correct_move_length) * avoid_move_scaler;
                feet_vertical_move += rg.position - GetClosestPointOnLine(rg.position, start_position, target.position);

                Debug.DrawLine(rg.position, rg.position + feet_vertical_move.normalized, Color.yellow);
            }
            else
            {
                feet_vertical_move = Vector3.MoveTowards(feet_vertical_move, Vector3.zero, back_speed * Time.fixedDeltaTime);
            }
                        
            Vector3 force = position_param * (now_target_position + feet_vertical_move - rg.position) + velocity_param * (Vector3.zero - rg.velocity);
            rg.AddForce(force, ForceMode.Acceleration);
            Debug.DrawLine(start_position, target.position, Color.green);
        }
    }
    Vector3 GetClosestPointOnLine(Vector3 point, Vector3 line_start, Vector3 line_end)
    {
        return line_start + Vector3.Project(point - line_start, line_end - line_start);
    }
}
