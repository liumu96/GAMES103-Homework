using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	float mass_inverse; 							// massInverse
	Matrix4x4 I_ref;							// reference inertia
	Matrix4x4 I_ref_inverse;					// the inverse of reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision

	float u_T = 0.5f;
	Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f); 

	Vector3[] vertices;


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;

		I_ref_inverse = I_ref.inverse;

		mass_inverse = 1/mass;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// dot product
	float Get_Dot_Product(Vector3 a, Vector3 b) {
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}

	// a matrix multiply a num
	Matrix4x4 Get_Matrix_Mul_Num(Matrix4x4 m, float num ) {
		Matrix4x4 A = Matrix4x4.zero;
		A[0, 0] = m[0, 0] * num;
		A[0, 1] = m[0, 1] * num;
		A[0, 2] = m[0, 2] * num;
		A[1, 0] = m[1, 0] * num;
		A[1, 1] = m[1, 1] * num;
		A[1, 2] = m[1, 2] * num;
		A[2, 0] = m[2, 0] * num;
		A[2, 1] = m[2, 1] * num;
		A[2, 2] = m[2, 2] * num;
		A[3, 3] = m[3, 3] * num;
		return A;
	}

	// the subtraction of two matrix
	Matrix4x4 Get_Sub_Two_Matrix(Matrix4x4 a, Matrix4x4 b) {
		Matrix4x4 A = Matrix4x4.zero;
		A[0, 0] = a[0, 0] - b[0, 0];
		A[0, 1] = a[0, 1] - b[0, 1];
		A[0, 2] = a[0, 2] - b[0, 2];
		A[1, 0] = a[1, 0] - b[1, 0];
		A[1, 1] = a[1, 1] - b[1, 1];
		A[1, 2] = a[1, 2] - b[1, 2];
		A[2, 0] = a[2, 0] - b[2, 0];
		A[2, 1] = a[2, 1] - b[2, 1];
		A[2, 2] = a[2, 2] - b[2, 2];
		A[3, 3] = a[3, 3] - b[3, 3];
		return A;
	}

	// the signed distance function for a point and a infinite plane
	float Get_Signed_Distance(Vector3 x, Vector3 p, Vector3 N) {
		return Get_Dot_Product(x - p, N);
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);

		int count = 0; // the collision vertices number
		Vector3 collisionV = new Vector3(0, 0, 0);  
		Vector3 collisionP = new Vector3(0, 0, 0); 
		Vector3 c = transform.position; // the position of the center of mass
		// Rigid Detection
		for(int i = 0; i < vertices.Length; i++) {
			// test if there is a collision
			Vector3 R_r_i = R * vertices[i];
			Vector3 x_i = c + R_r_i;
			float distance = Get_Signed_Distance(x_i, P, N); // signed distance
			
			if(distance < 0) {
				Vector3 w_R_i = Get_Cross_Matrix(w) * R_r_i;
				Vector3 v_i = v + w_R_i;
				if (Get_Dot_Product(v_i, N) < 0) {
					collisionV += v_i;
					collisionP += R_r_i;
					count++;
				}
			}
		}
		// Rigid Response
		if(count > 0) {
			// average these collision vertices
			collisionV /= count;
			collisionP /= count;

			// compute the v_i_new
			Vector3 v_N = Get_Dot_Product(collisionV, N) * N;
			Vector3 v_T = collisionV - v_N;
			// compute the friction attenuation
			float a = Mathf.Max(0, 1 - u_T * (1 + restitution) * v_N.magnitude / v_T.magnitude, 0);
			Vector3 v_N_new = - restitution * v_N;
			Vector3 v_T_new = - a * v_T;
			Vector3 v_new = v_N_new + v_T_new;

			// compute the impulse J
			Matrix4x4 R_r_i_c = Get_Cross_Matrix(collisionP);
			Matrix4x4 I = Matrix4x4.identity;
			Matrix4x4 K = Get_Sub_Two_Matrix(Get_Matrix_Mul_Num(I, mass_inverse), R_r_i_c * I_ref_inverse * R_r_i_c);
			Vector3 J = K.inverse * (v_new - collisionV);
			// update v and w
			v = v + mass_inverse * J;
			Vector3 d_w = I_ref_inverse * R_r_i_c * J;
			w = w + d_w;
			// decrease the restitution to reduce oscillation.
			restitution *= 0.5f;
		}

	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}
		if(launched) {
			// Part I: Update velocities
			v = linear_decay * (v + dt * gravity);
			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x = transform.position;
			x += dt * v;
			//Update angular status
			Quaternion q_0 = transform.rotation;
			Vector3 w_d = w * dt * 0.5f;
			Quaternion q_w = new Quaternion(w_d.x, w_d.y, w_d.z, 0.0f);
			Quaternion temp = q_w * q_0;
			Quaternion q = new Quaternion(q_0.x + temp.x, q_0.y + temp.y, q_0.z + temp.z, q_0.w + temp.w);

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}

		
	}
}
