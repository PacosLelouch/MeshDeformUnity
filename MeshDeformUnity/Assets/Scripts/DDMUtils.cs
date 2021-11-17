using UnityEngine;
using System;
using MathNet.Numerics.LinearAlgebra;

public class DDMUtils
{
	int n;
	float dm_blend;
	int num_transforms;
	MathNet.Numerics.LinearAlgebra.Single.SparseMatrix C;
	MathNet.Numerics.LinearAlgebra.Single.SparseMatrix W;
	MathNet.Numerics.LinearAlgebra.Single.SparseMatrix B;
	MathNet.Numerics.LinearAlgebra.Single.SparseMatrix V;


	public Matrix4x4 compute_psi(int i, int j)
	{
		Matrix4x4 res = new Matrix4x4();

		for (int k = 0; k < n; k++)
		{
			float w = W[k, j];
			float b = B[k, j];
			if (w != 0 && b != 0)
			{
				Vector4 rw = new Vector4(V[k, 0], V[k, 1], V[k, 2], 1);
				for (int x = 0; x < 4; x++)
				{
					for (int y = 0; y < 4; y++)
					{
						res[x, y] += rw[x] * rw[y] * b * w;
					}
				}
			}
		}

		return res;
	}

	public Vector3 compute_pi(int i)
	{
		Matrix4x4 sum = new Matrix4x4();
		
		for (int k = 0; k < num_transforms; k++)
		{
			Matrix4x4 psi = compute_psi(i, k);
			for (int t = 0; t < 16; t++)
			{
				sum[t] += psi[t];
			}
		}

		Vector3 res = new Vector3(sum[0, 3], sum[1, 3], sum[2, 3]);

		return res;
	}

	public float compute_w_prime(int i, int j)
	{
		float res = 0;
		for (int k = 0; k < n; k++)
		{
			float w = W[k, j];
			float c = C[k, i];
			res += w * c;
		}

		return res;
	}

	public Matrix4x4 compute_omega(int i, int j)
	{
		Vector3 p_i = compute_pi(num_transforms, i);
		Matrix4x4 p_i_mat = new Matrix4x4();
		for (int x = 0; x < 3; x++)
		{
			for (int y = 0; y < 3; y++)
			{
				p_i_mat[x, y] = p_i[x] * p_i[y];
			}
		}
		p_i_mat.SetColumn(3, new Vector4(p_i.x, p_i.y, p_i.z, 1));
		p_i_mat.SetRow(3, new Vector4(p_i.x, p_i.y, p_i.z, 1));

		Matrix4x4 psi_ij = compute_psi(n, i, j);

		Matrix4x4 res = new Matrix4x4();
		for (int k = 0; k < 16; k++)
		{
			res[k] = (1.0f - dm_blend) * psi_ij[k] + dm_blend * compute_w_prime(i, j) * p_i_mat[k];
		}

		return res;
	}
}