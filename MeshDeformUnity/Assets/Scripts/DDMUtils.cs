using UnityEngine;
using System;
using MathNet.Numerics.LinearAlgebra.Single;

public class DDMUtils
{
	public int n;
	public float dm_blend;
	public int num_transforms;
	public SparseMatrix C;
	public DenseMatrix W;
	public SparseMatrix B;
	public DenseMatrix V;

	private DenseMatrix[,] psis;
	private bool[,] psisValid;

	private float[,] wps;
	private bool[,] wpsValid;

	private DenseVector[] ps;
	private bool[] psValid;

	public void InitCache()
    {
		psis = new DenseMatrix[n, num_transforms];
		psisValid = new bool[n, num_transforms];

		wps = new float[n, num_transforms];
		wpsValid = new bool[n, num_transforms];

		ps = new DenseVector[n];
		psValid = new bool[n];

		for (int vi = 0; vi < n; ++vi)
        {
			for(int bi = 0; bi < num_transforms; ++bi)
			{
				//psis[vi, bi] = (new DenseMatrix(4));
				psisValid[vi, bi] = false;

				//wps[vi, bi] = 0.0f;
				wpsValid[vi, bi] = false;
			}
			//ps[vi] = (new DenseVector(3));
			psValid[vi] = false;
		}
	}

	public DenseMatrix compute_psi(int i, int j)
	{
		if(psisValid[i, j])
        {
			return psis[i, j];
        }
		DenseMatrix res = new DenseMatrix(4);

		for (int k = 0; k < n; k++)
		{
			float w = W[k, j];
			float b = B[k, i];
			if (w != 0 && b != 0)
			{
				float[] v = new float[] { V[k, 0], V[k, 1], V[k, 2], 1.0f };
				DenseVector rw = new DenseVector(v);
				DenseMatrix hh = new DenseMatrix(4);
				rw.OuterProduct(rw, hh);
				hh *= (b * w);
				res += hh;
				//for (int x = 0; x < 4; x++)
				//{
				//	for (int y = 0; y < 4; y++)
				//	{
				//		res[x, y] += hh[x, y];
				//	}
				//}
			}
		}

		psis[i, j] = res;
		psisValid[i, j] = true;
		return res;
	}

	public DenseVector compute_pi(int i)
	{
		if (psValid[i])
		{
			return ps[i];
		}
		DenseVector sum = new DenseVector(3);
		
		for (int k = 0; k < num_transforms; k++)
		{
			DenseMatrix psi = compute_psi(i, k);
			sum[0] += psi[0, 3];
			sum[1] += psi[1, 3];
			sum[2] += psi[2, 3];
			//Matrix4x4 psi = compute_psi(i, k);
			//for (int t = 0; t < 16; t++)
			//{
			//	sum[t] += psi[t];
			//}
		}

		//Vector3 res = new Vector3(sum[0, 3], sum[1, 3], sum[2, 3]);

		ps[i] = sum;
		psValid[i] = true;
		return sum;
	}

	public float compute_w_prime(int i, int j)
	{
		if (wpsValid[i, j])
		{
			return wps[i, j];
		}

		float res = 0;
		for (int k = 0; k < n; k++)
		{
			float w = W[k, j];
			float c = C[k, i];
			res += w * c;
		}

		wps[i, j] = res;
		wpsValid[i, j] = true;
		return res;
	}

	public DenseMatrix compute_omega(int i, int j)
	{
		DenseVector p_i = compute_pi(i);
		DenseMatrix p_i_mat = new DenseMatrix(3);
		p_i.OuterProduct(p_i, p_i_mat);
		//for (int x = 0; x < 3; x++)
		//{
		//	for (int y = 0; y < 3; y++)
		//	{
		//		p_i_mat[x, y] = p_i[x] * p_i[y];
		//	}
		//}
		//p_i_mat.SetColumn(3, new Vector4(p_i.x, p_i.y, p_i.z, 1));
		//p_i_mat.SetRow(3, new Vector4(p_i.x, p_i.y, p_i.z, 1));
		DenseMatrix p_i_mat_4 = new DenseMatrix(4);
		p_i_mat_4.SetSubMatrix(0, 0, p_i_mat);

		DenseMatrix psi_ij = compute_psi(i, j);

		return (1.0f - dm_blend) * psi_ij + (dm_blend * compute_w_prime(i, j)) * p_i_mat_4;

		//Matrix4x4 res = new Matrix4x4();
		//for (int k = 0; k < 16; k++)
		//{
		//	res[k] = (1.0f - dm_blend) * psi_ij[k] + dm_blend * compute_w_prime(i, j) * p_i_mat[k];
		//}

		//return res;
	}
}