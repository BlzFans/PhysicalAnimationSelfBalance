using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelfBalanceMonster : MonoBehaviour
{
    /// <summary>
    /// Joint of feet will use for reach the IK target
    /// </summary>
    public MonsterBalanceFeet[] feets;
    public Vector3[] poles_world_position;
    [Header("Self balance parameters")]
    public float feet_move_speed = 3f;
    public float base_feet_radius = 0.4f;
    public float gait_max_height = 0.3f;
    public AnimationCurve gait_curve;
    public float keep_feet_back_speed = 3f;
    public float keep_root_up_speed = 5f;
    public float min_root_back_speed = 0.5f;
    public float keep_root_back_scale = 3f;
    public float over_feet_scale = 0.5f;
    [Range(0f, 1f)]
    public float next_feet_move_interval = 0.5f;
    [Header("External parameters")]
    public float external_acceleration = 10f;
    public Transform target;
    public Transform root;
    public Transform pole;
    MonsterPhysicalLeg[] legs;
    Rigidbody root_rg;
    Rigidbody body_rg;
    List<Rigidbody> all_rgs;
    Transform center_of_all_mass;
    Vector3 now_root_velocity;
    const int max_feet_moveing_num = 2;
    List<int> moveing_leg_indexes = new List<int>();
    List<int> backing_leg_indexes = new List<int>();
    Vector3[] start_feet_bias;
    float[] now_gait_duration;
    float[] now_gait_duration_start;
    float[] now_gait_duration_scale;

    // Debug 
    List<Vector3> draw_triangles = new List<Vector3>();
    List<Tuple<Vector3, Vector3>> draw_lines = new List<Tuple<Vector3, Vector3>>();
    List<Color> lines_colors = new List<Color>();
    List<List<Vector3>> draw_trajectorys = new List<List<Vector3>>();
    List<Vector3> draw_target = new List<Vector3>();
    // Start is called before the first frame update
    void Start()
    {
        root_rg = GetComponent<Rigidbody>();
        if (transform.parent != null)
            all_rgs = transform.parent.GetComponentsInChildren<Rigidbody>().ToList();
        else
            all_rgs = transform.GetComponentsInChildren<Rigidbody>().ToList();
        
        center_of_all_mass = new GameObject("CoM").transform;
        CalculateCenterOfMass();

        // Get all legs joints
        legs = new MonsterPhysicalLeg[feets.Length];
        for (int i = 0; i < feets.Length; i++)
        {
            MonsterPhysicalLeg new_leg = new MonsterPhysicalLeg();
            ConfigurableJoint leg_joint = feets[i].GetComponent<ConfigurableJoint>();
            while (leg_joint != null)
            {
                new_leg.joints.Add(leg_joint);
                leg_joint = leg_joint.connectedBody.GetComponent<ConfigurableJoint>();
            }
            // Init all legs parameters
            new_leg.InitParameters(center_of_all_mass, poles_world_position[i]);
            legs[i] = new_leg;
        }

        body_rg = GameObject.Find("Body").GetComponent<Rigidbody>();

        start_feet_bias = new Vector3[feets.Length];
        now_gait_duration = new float[feets.Length];
        now_gait_duration_start = new float[feets.Length];
        now_gait_duration_scale = new float[feets.Length];
    }
    void FixedUpdate()
    {
        // Debug
        draw_lines = new List<Tuple<Vector3, Vector3>>();
        lines_colors = new List<Color>();
        draw_triangles = new List<Vector3>();
        draw_trajectorys = new List<List<Vector3>>();
        draw_target = new List<Vector3>();

        Vector3 direction = new Vector3();
        if (Input.GetKey("w"))
        {
            direction += new Vector3(0f, 0f, 1f);
        }
        if (Input.GetKey("s"))
        {
            direction += new Vector3(0f, 0f, -1f);
        }
        if (Input.GetKey("a"))
        {
            direction += new Vector3(-1f, 0f, 0f);
        }
        if (Input.GetKey("d"))
        {
            direction += new Vector3(1f, 0f, 0f);
        }
        direction = direction.normalized;
        body_rg.AddForce(direction * external_acceleration, ForceMode.Acceleration);

        SelfBalance();

        // Debug
        //LimbIKMoveToTargrtPoint(legs[0], target.position, root.position, pole.position);
    }
    void CalculateCenterOfMass()
    {
        Vector3 CoM = Vector3.zero;
        float c = 0f;
        foreach (var rg in all_rgs)
        {
            CoM += rg.worldCenterOfMass * rg.mass;
            c += rg.mass;
        }
        CoM /= c;

        center_of_all_mass.position = CoM;
        center_of_all_mass.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(root_rg.transform.forward, Vector3.up), Vector3.up);
        now_root_velocity = Vector3.ProjectOnPlane(root_rg.velocity, Vector3.up);

        // Debug
        draw_lines.Add(new Tuple<Vector3, Vector3>(CoM, CoM - Vector3.up * 3f));
        lines_colors.Add(Color.black);
    }
    float GetRootAngle()
    {
        return root_rg.transform.localEulerAngles.x < 180f ? root_rg.transform.localEulerAngles.x : root_rg.transform.localEulerAngles.x - 360f;
    }
    float GetMinDuration()
    {
        float min_duration = 1f;
        foreach (int i in moveing_leg_indexes)
            if (now_gait_duration[i] < min_duration) min_duration = now_gait_duration[i];
        return min_duration;
    }
    void SelfBalance()
    {
        CalculateCenterOfMass();
        foreach (var feet in feets)
            feet.FeetFixedUpdate();

        if (moveing_leg_indexes.Count < max_feet_moveing_num && GetMinDuration() >= next_feet_move_interval)
        {
            int moveing_leg_index = -1;
            float max_magnitude = 0f;
            bool max_feet_backing = true;
            Vector3 feet_move_direction = Vector3.zero;
            for (int i = 0; i < legs.Length; i++)
            {
                if (!moveing_leg_indexes.Contains(i))
                {
                    feet_move_direction = legs[i].GetNowJointPosition(0) - center_of_all_mass.TransformPoint(legs[i].base_feet_bias);
                    feet_move_direction = Vector3.ProjectOnPlane(feet_move_direction, Vector3.up);
                    if (feet_move_direction.magnitude > base_feet_radius)
                    {
                        bool is_feet_backing = backing_leg_indexes.Contains(i);
                        if ((feet_move_direction.magnitude > max_magnitude && max_feet_backing == is_feet_backing) || max_feet_backing && !is_feet_backing)
                        {
                            max_magnitude = feet_move_direction.magnitude;
                            max_feet_backing = is_feet_backing;
                            moveing_leg_index = i;
                        }                        
                    }
                }
            }

            if (moveing_leg_index > -1)
            {
                start_feet_bias[moveing_leg_index] = legs[moveing_leg_index].GetNowFeetBias(center_of_all_mass);
                moveing_leg_indexes.Add(moveing_leg_index);
            }
        }

        for (int i = 0; i < legs.Length; i++)
        {
            Vector3 now_target_point, now_root_point, now_pole_point;
            Vector3 now_feet_point = legs[i].GetNowJointPosition(0);
            (now_target_point, now_root_point, now_pole_point) = legs[i].GetWorldBiasPosition(center_of_all_mass);
            now_target_point.y = feets[i].garound_height;
            now_root_point.y = feets[i].garound_height + legs[i].base_root_height_bias;
            now_pole_point.y = feets[i].garound_height + legs[i].base_pole_height_bias;
            now_feet_point.y = feets[i].garound_height;
            // Damping the root velocity
            if (now_root_velocity.magnitude > min_root_back_speed)
                now_root_point -= now_root_velocity * keep_root_back_scale;

            if (moveing_leg_indexes.Contains(i))
            {
                now_target_point = PlanNowTargetPoint(legs[i], now_target_point, i);
            }
            else if (backing_leg_indexes.Contains(i))
            {
                if ((now_target_point - now_feet_point).magnitude <= base_feet_radius) 
                    backing_leg_indexes.Remove(i);
                else
                    now_target_point = Vector3.MoveTowards(now_feet_point, now_target_point, keep_feet_back_speed * Time.fixedDeltaTime);
            } 
            else
            {
                now_target_point = now_feet_point;
            }
            LimbIKMoveToTargrtPoint(legs[i], now_target_point, now_root_point, now_pole_point);
            
            // Debug
            Vector3 v = center_of_all_mass.TransformPoint(legs[i].base_feet_bias);
            v.y = feets[i].garound_height;
            draw_target.Add(v);
        }
    }
    Vector3 PlanNowTargetPoint(MonsterPhysicalLeg leg, Vector3 target_feet_point, int feet_index)
    {
        //
        target_feet_point += now_root_velocity * over_feet_scale;

        Vector3 start_feet_point = center_of_all_mass.TransformPoint(start_feet_bias[feet_index]);
        Vector3 feet_move_direction = target_feet_point - start_feet_point;
        feet_move_direction = Vector3.ProjectOnPlane(feet_move_direction, Vector3.up);

        Vector3 now_feet_move_length = feet_move_direction.normalized * feet_move_speed * Time.fixedDeltaTime;

        Vector3 now_feet_position = leg.GetNowJointPosition(0);
        Vector3 feet_already_move_length = Vector3.Project(now_feet_position - start_feet_point, feet_move_direction);
        if (Vector3.Dot(feet_already_move_length, feet_move_direction) < 0f) feet_already_move_length = Vector3.zero;

        if (now_gait_duration[feet_index] == 0f)
        {
            while (now_gait_duration[feet_index] < 0.5f && Mathf.Lerp(feets[feet_index].garound_height, feets[feet_index].garound_height + gait_max_height, gait_curve.Evaluate(now_gait_duration[feet_index])) < start_feet_point.y)
            {
                now_gait_duration[feet_index] += 0.01f;
            }
            now_gait_duration_start[feet_index] = now_gait_duration[feet_index];
            now_gait_duration_scale[feet_index] = 1f - now_gait_duration_start[feet_index];
        }

        float new_gait_duration = Mathf.Max(now_gait_duration_start[feet_index] + Mathf.InverseLerp(0f, feet_move_direction.magnitude, feet_already_move_length.magnitude) * now_gait_duration_scale[feet_index],
            now_gait_duration[feet_index] + Mathf.InverseLerp(0f, feet_move_direction.magnitude, now_feet_move_length.magnitude));
        if (new_gait_duration >= 1f)
        {
            now_feet_move_length *= (1f - now_gait_duration[feet_index]) / (new_gait_duration - now_gait_duration[feet_index]);
            now_gait_duration[feet_index] = 0f;
            moveing_leg_indexes.Remove(feet_index);
            if (!backing_leg_indexes.Contains(feet_index))
            {
                backing_leg_indexes.Add(feet_index);
                if (backing_leg_indexes.Count > max_feet_moveing_num) backing_leg_indexes.RemoveAt(0);
            }
        }
        else
        {
            now_gait_duration[feet_index] = new_gait_duration;
        }
        Vector3 now_target_point = start_feet_point + feet_move_direction * now_gait_duration[feet_index] + now_feet_move_length;
        now_target_point.y = Mathf.Lerp(feets[feet_index].garound_height, Mathf.Max(feets[feet_index].garound_height + gait_max_height, now_feet_position.y), gait_curve.Evaluate(now_gait_duration[feet_index]));

        // Debug
        if (now_gait_duration[feet_index] > 0f)
        {
            var trajectory = new List<Vector3>();
            for (float t = now_gait_duration[feet_index]; t <= 1f; t += 0.1f)
            {
                Vector3 v = start_feet_point + feet_move_direction * t;
                v.y = Mathf.Lerp(feets[feet_index].garound_height, Mathf.Max(feets[feet_index].garound_height + gait_max_height, now_feet_position.y), gait_curve.Evaluate(t));
                trajectory.Add(v);
            }
            draw_trajectorys.Add(trajectory);
        }        

        return now_target_point;
    }

    void LimbIKMoveToTargrtPoint(MonsterPhysicalLeg leg, Vector3 now_target_point, Vector3 now_root_point, Vector3 now_pole_point)
    {
        ConfigurableJoint joint_u = leg.joints[2];
        ConfigurableJoint joint_l = leg.joints[1];
        float upper_leg_length = leg.link_length[1];
        float lower_leg_length = leg.link_length[0];

        // Get base height of upper joint for character to maintain
        Vector3 joint_pos_u = joint_u.connectedBody.transform.TransformPoint(joint_u.connectedAnchor);
        joint_pos_u = Vector3.MoveTowards(joint_pos_u, now_root_point, keep_root_up_speed * Time.fixedDeltaTime);

        Vector3 ik_plane_normal = Vector3.Cross(now_target_point - joint_pos_u, now_pole_point - joint_pos_u);

        Vector3 target_c = now_target_point - joint_pos_u;
        float max_leg_extend_length = upper_leg_length + lower_leg_length;
        if (target_c.magnitude > max_leg_extend_length)
            target_c = target_c.normalized * max_leg_extend_length;

        float angle_u = LawOfCosines(upper_leg_length, target_c.magnitude, lower_leg_length);
        Vector3 target_a = Quaternion.AngleAxis(angle_u, ik_plane_normal) * target_c.normalized * upper_leg_length;

        Vector3 joint_pos_l = joint_pos_u + target_a;
        Vector3 target_b = now_target_point - joint_pos_l;

        SetJointPDTarget(joint_u, target_a.normalized, ik_plane_normal, joint_pos_u, leg.joints_axis[1]);
        SetJointPDTarget(joint_l, target_b.normalized, ik_plane_normal, joint_pos_l, leg.joints_axis[0]);

        // Debug
        Vector3 ik_plane_center = (joint_pos_l + joint_pos_u + now_target_point) / 3f;
        draw_lines.Add(new Tuple<Vector3, Vector3>(ik_plane_center, ik_plane_center + ik_plane_normal));
        lines_colors.Add(Color.white);

        draw_triangles.Add(joint_pos_u);
        draw_triangles.Add(joint_pos_u + target_a);
        draw_triangles.Add(now_target_point);        
        
    }
    void SetJointPDTarget(ConfigurableJoint joint, Vector3 link_direction, Vector3 ik_plane_normal, Vector3 joint_pos, Tuple<Vector3, Vector3> joint_axis)
    {
        // Get joint rotation in joint space
        Vector3 x_axis = joint.connectedBody.transform.TransformDirection(joint_axis.Item1);
        Vector3 y_axis = joint.connectedBody.transform.TransformDirection(joint_axis.Item2);        
        Vector3 z_axis = Vector3.Cross(x_axis, y_axis);
        Quaternion axis_rotation = Quaternion.LookRotation(z_axis, y_axis);

        Vector3 now_z_axis = Vector3.Cross(ik_plane_normal, -link_direction);
        Quaternion now_link_rotation = Quaternion.LookRotation(now_z_axis, -link_direction);
        Quaternion link_local_rotation = Quaternion.Inverse(now_link_rotation) * axis_rotation;
        // Set PD controller's target
        joint.targetRotation = link_local_rotation;

        // Debug
        draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos, joint_pos + z_axis));
        lines_colors.Add(Color.blue);
        draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos, joint_pos + x_axis));
        lines_colors.Add(Color.red);
        draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos, joint_pos + y_axis));
        lines_colors.Add(Color.green);

        draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos, joint_pos + now_z_axis));
        lines_colors.Add(Color.blue / 2f);
        draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos, joint_pos + ik_plane_normal));
        lines_colors.Add(Color.red / 2f);
        draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos, joint_pos + -link_direction));
        lines_colors.Add(Color.green / 2f);
    }
    float LawOfCosines(float a, float b, float c)
    {
        /*
               c
           B ______ A
             \    /
            a \  / b
               \/
               C
        */
        float cosC = (Mathf.Pow(a, 2f) + Mathf.Pow(b, 2f) - Mathf.Pow(c, 2f)) / (2f * a * b);
        float C = Mathf.Acos(Mathf.Clamp(cosC, -1f, 1f)) * Mathf.Rad2Deg;
        return C;
    }
    void OnDrawGizmos()
    {
        // Draw IK triangles
        for (int i = 0; i < draw_triangles.Count; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(draw_triangles[i], draw_triangles[(i+1) % 3 == 0 ? i-2 : i+1]);
            Gizmos.DrawSphere(draw_triangles[i], 0.05f);
        }
        // Draw line
        for (int i = 0; i < draw_lines.Count; i++)
        {
            Gizmos.color = lines_colors[i];
            Gizmos.DrawLine(draw_lines[i].Item1, draw_lines[i].Item2);
        }
        // Draw trajectory
        foreach (var trajectory in draw_trajectorys)
        {
            for (int i = 0; i < trajectory.Count; i++)
            {
                Gizmos.color = Color.yellow;
                if (i > 0)
                {
                    Gizmos.DrawLine(trajectory[i], trajectory[i-1]);
                }
                Gizmos.DrawSphere(trajectory[i], 0.025f);
            }
        }
        // Draw target
        foreach (Vector3 target in draw_target)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(target, base_feet_radius);
        }
    }
}
