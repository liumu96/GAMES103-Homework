using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
	public bool launched = false;
	Vector3[] X;
	Vector3[] Q;
	Vector3[] V;
	Matrix4x4 QQt = Matrix4x4.zero;

	float linear_decay	= 0.999f;				// for velocity decay
	float restitution 	= 0.5f;					// for collision
	float u_T = 0.5f;
	float coeff = 1.3f; 
	float epsilon = 0.03f;
	Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f); 



    // Start is called before the first frame update
    void Start()
    {
    	Mesh mesh = GetComponent<MeshFilter>().mesh;
        V = new Vector3[mesh.vertices.Length];
        X = mesh.vertices;
        Q = mesh.vertices;

        //Centerizing Q.
        Vector3 c=Vector3.zero;
        for(int i=0; i<Q.Length; i++)
        	c+=Q[i];
        c/=Q.Length;
        for(int i=0; i<Q.Length; i++)
        	Q[i]-=c;

        //Get QQ^t ready.
		for(int i=0; i<Q.Length; i++)
		{
			QQt[0, 0]+=Q[i][0]*Q[i][0];
			QQt[0, 1]+=Q[i][0]*Q[i][1];
			QQt[0, 2]+=Q[i][0]*Q[i][2];
			QQt[1, 0]+=Q[i][1]*Q[i][0];
			QQt[1, 1]+=Q[i][1]*Q[i][1];
			QQt[1, 2]+=Q[i][1]*Q[i][2];
			QQt[2, 0]+=Q[i][2]*Q[i][0];
			QQt[2, 1]+=Q[i][2]*Q[i][1];
			QQt[2, 2]+=Q[i][2]*Q[i][2];
		}
		QQt[3, 3]=1;

		for(int i=0; i<X.Length; i++)
			V[i][0]=4.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position=Vector3.zero;
		transform.rotation=Quaternion.identity;
   	}

   	// Polar Decomposition that returns the rotation from F.
   	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C[ii,jj]+=F[kk,ii]*F[kk,jj];
	   
	   	Matrix4x4 C2 = Matrix4x4.zero;
		for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C2[ii,jj]+=C[ii,kk]*C[jj,kk];
	    
	    float det    =  F[0,0]*F[1,1]*F[2,2]+
	                    F[0,1]*F[1,2]*F[2,0]+
	                    F[1,0]*F[2,1]*F[0,2]-
	                    F[0,2]*F[1,1]*F[2,0]-
	                    F[0,1]*F[1,0]*F[2,2]-
	                    F[0,0]*F[1,2]*F[2,1];
	    
	    float I_c    =   C[0,0]+C[1,1]+C[2,2];
	    float I_c2   =   I_c*I_c;
	    float II_c   =   0.5f*(I_c2-C2[0,0]-C2[1,1]-C2[2,2]);
	    float III_c  =   det*det;
	    float k      =   I_c2-3*II_c;
	    
	    Matrix4x4 inv_U = Matrix4x4.zero;
	    if(k<1e-10f)
	    {
	        float inv_lambda=1/Mathf.Sqrt(I_c/3);
	        inv_U[0,0]=inv_lambda;
	        inv_U[1,1]=inv_lambda;
	        inv_U[2,2]=inv_lambda;
	    }
	    else
	    {
	        float l = I_c*(I_c*I_c-4.5f*II_c)+13.5f*III_c;
	        float k_root = Mathf.Sqrt(k);
	        float value=l/(k*k_root);
	        if(value<-1.0f) value=-1.0f;
	        if(value> 1.0f) value= 1.0f;
	        float phi = Mathf.Acos(value);
	        float lambda2=(I_c+2*k_root*Mathf.Cos(phi/3))/3.0f;
	        float lambda=Mathf.Sqrt(lambda2);
	        
	        float III_u = Mathf.Sqrt(III_c);
	        if(det<0)   III_u=-III_u;
	        float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2*III_u/lambda);
	        float II_u=(I_u*I_u-I_c)*0.5f;
	        
	        
	        float inv_rate, factor;
	        inv_rate=1/(I_u*II_u-III_u);
	        factor=I_u*III_u*inv_rate;
	        
	       	Matrix4x4 U = Matrix4x4.zero;
			U[0,0]=factor;
	        U[1,1]=factor;
	        U[2,2]=factor;
	        
	        factor=(I_u*I_u-II_u)*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            U[i,j]+=factor*C[i,j]-inv_rate*C2[i,j];
	        
	        inv_rate=1/III_u;
	        factor=II_u*inv_rate;
	        inv_U[0,0]=factor;
	        inv_U[1,1]=factor;
	        inv_U[2,2]=factor;
	        
	        factor=-I_u*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            inv_U[i,j]+=factor*U[i,j]+inv_rate*C[i,j];
	    }
	    
	    Matrix4x4 R=Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        R[ii,jj]+=F[ii,kk]*inv_U[kk,jj];
	    R[3,3]=1;
	    return R;
	}

	float Dot_Product(Vector3 a, Vector3 b)
    {
        //3x3向量点积
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
   	{
   		for(int i=0; i<Q.Length; i++)
		{
			Vector3 x=(Vector3)(R*Q[i])+c;

			V[i]+=(x-X[i])*inv_dt;
			X[i]=x;
		}	
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices=X;
   	}

	void Collision(float inv_dt, Vector3 P, Vector3 N) {
		for (int i = 0; i < V.Length; i++) {
			Vector3 x_p = X[i] - P;
			float X_proj = Dot_Product(x_p, N);
			if(X_proj < epsilon) {
				if(Dot_Product(V[i], N) < 0) {
					Vector3 V_N = Dot_Product(V[i], N) * N;
					Vector3 V_T = V[i] - V_N;
					float a = Mathf.Max(0, 1 - u_T * (1 + restitution) * V_N.magnitude / V_T.magnitude);
					Vector3 V_N_new = -restitution * V_N;
					Vector3 V_T_new = a * V_T;
					V[i] = V_N_new + V_T_new;
					X[i] = X[i] + Mathf.Abs(X_proj) * N * coeff;
					restitution *= 0.5f;
				}
			}
		}

	}

    // Update is called once per frame
    void Update()
    {
  		float dt = 0.015f;

		if(Input.GetKey("r"))
		{
			transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
			Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
			transform.position=Vector3.zero;
			transform.rotation = Quaternion.identity;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			launched=true;
			for(int i = 0; i < V.Length; i++) {
				V[i] = new Vector3 (4.0f, 0.0f, 0.0f);
			}
		}
		if(!launched) return;

  		//Step 1: run a simple particle system.
        for(int i=0; i<V.Length; i++)
        {
			V[i] = linear_decay * (V[i] + dt * gravity);
			X[i] += dt * V[i];
        }

        //Step 2: Perform simple particle collision.
		Collision(1/dt, new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision(1/dt, new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Step 3: Use shape matching to get new translation c and 
		// new rotation R. Update the mesh by c and R.
        //Shape Matching (translation)
		Vector3 c = new Vector3(0, 0, 0);
		for(int i = 0; i < X.Length; i++) {
			c += X[i];
		}
		c = c / V.Length;
		
		//Shape Matching (rotation)
		Matrix4x4 A = Matrix4x4.zero;
		for(int i = 0; i < Q.Length; i+=5) {
			Vector3 r_i = X[i] - c;
			A[0, 0] += r_i[0] * Q[i][0];
			A[0, 1] += r_i[0] * Q[i][1];
			A[0, 2] += r_i[0] * Q[i][2];
			A[1, 0] += r_i[1] * Q[i][0];
			A[1, 1] += r_i[1] * Q[i][1];
			A[1, 2] += r_i[1] * Q[i][2];
			A[2, 0] += r_i[2] * Q[i][0];
			A[2, 1] += r_i[2] * Q[i][1];
			A[2, 2] += r_i[2] * Q[i][2];
		}
		A[3,3] = 1;
		A *= QQt.inverse;
		Matrix4x4 R = Get_Rotation(A);
		
		Update_Mesh(c, R, 1/dt);
    }
}
