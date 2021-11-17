using UnityEngine;
using UnityEngine.Profiling;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using MathNet.Numerics.LinearAlgebra.Single.Solvers;

/*
	Vertecx adjacency functions
*/
public class MeshUtils// : MonoBehaviour 
{
	const float EPSILON = 1e-8f;

	// Collect information about vertex adjacency into matrix
	public static int[,] BuildAdjacencyMatrix (Vector3[] v, int[] t, int maxNeighbors, float minSqrDistance = EPSILON)
	{
		Profiler.BeginSample("BuildAdjacencyMatrix");
		var adj = new int[v.Length, maxNeighbors];
		for (int i = 0; i < adj.GetLength(0); ++i)
			for (int j = 0; j < adj.GetLength(1); ++j)
				adj[i, j] = -1;

		int[] mapToUnique = MapVerticesToUniquePositions(v, minSqrDistance);

		for(int tri = 0; tri < t.Length; tri = tri + 3)
		{
			AddEdgeToAdjacencyMatrix(ref adj, mapToUnique, t[tri  ], t[tri+1]);
			AddEdgeToAdjacencyMatrix(ref adj, mapToUnique, t[tri  ], t[tri+2]);
			AddEdgeToAdjacencyMatrix(ref adj, mapToUnique, t[tri+1], t[tri+2]);
		}

		BroadcastAdjacencyFromUniqueToAllVertices(ref adj, mapToUnique);

		Profiler.EndSample();
		return adj;
	}

	// Find vertices that approximately share the same positions
	// Returns array of indices pointing to the first occurance of particular position in the vertex array
	public static int[] MapVerticesToUniquePositions (Vector3[] v, float minSqrDistance = EPSILON)
	{
		Profiler.BeginSample("MapVerticesToUniquePositions");

		var mapToUnique = new int[v.Length];
		for (int i = 0; i < mapToUnique.Length; ++i)
			mapToUnique[i] = -1;

		for (int i = 0; i < v.Length; i++)
			for (int j = i; j < v.Length; j++)
				if (mapToUnique[j] == -1) // skip, if already pointing to unique position
				{
					var u = mapToUnique[i];
					if (u == -1)
						u = i;
						
					var dx = v[u].x - v[j].x;
					var dy = v[u].y - v[j].y;
					var dz = v[u].z - v[j].z;
					//if (Vector3.Distance(v[i], v[j]) < minDistance) // 2794ms
					//if ((v[j] - v[i]).sqrMagnitude < minSqrDistance) // 2796ms
					if (dx*dx+dy*dy+dz*dz < minSqrDistance) // 687ms
					{
						if (mapToUnique[i] == -1)
							mapToUnique[i] = u; // found new unique vertex
						mapToUnique[j] = u;
					}
				}

		for (int i = 0; i < v.Length; i++)
			Debug.Assert(mapToUnique[i] != -1);

		Profiler.EndSample();
		return mapToUnique;
	}

	private static void AddVertexToAdjacencyMatrix(ref int[,] adjacencyMatrix, int from, int to)
	{
		Profiler.BeginSample("AddVertexToAdjacencyMatrix");
		var maxNeighbors = adjacencyMatrix.GetLength(1);
		for (int i = 0; i < maxNeighbors; i++)
		{
			if (adjacencyMatrix[from, i] == to)
				break;
			
			if (adjacencyMatrix[from, i] == -1)
			{
				adjacencyMatrix[from, i] = to;
				break;
			}
		}
		Profiler.EndSample();
	}

	private static void AddEdgeToAdjacencyMatrix(ref int[,] adjacencyMatrix, int[] mapToUnique, int v0, int v1)
	{
		var u0 = mapToUnique[v0];
		var u1 = mapToUnique[v1];

		AddVertexToAdjacencyMatrix(ref adjacencyMatrix, u0, u1);
		AddVertexToAdjacencyMatrix(ref adjacencyMatrix, u1, u0);
	}

	private static void BroadcastAdjacencyFromUniqueToAllVertices (ref int[,] adjacencyMatrix, int[] mapToUnique)
	{
		Profiler.BeginSample("BroadcastAdjacencyFromUniqueToAllVertices");

		var maxNeighbors = adjacencyMatrix.GetLength(1);
		Debug.Assert(adjacencyMatrix.GetLength(0) == mapToUnique.Length);

		for (int i = 0; i < mapToUnique.Length; ++i)
		{
			var u = mapToUnique[i];
			if (u == i)
				continue;

			Debug.Assert(adjacencyMatrix[i, 0] == -1);
			for (int j = 0; j < maxNeighbors && adjacencyMatrix[u, j] != -1; ++j)
				adjacencyMatrix[i, j] = adjacencyMatrix[u, j];
		}

		Profiler.EndSample();
	}

	/// <summary>
	/// Build Laplacian matrix from adjacent matrix.
	/// </summary>
	/// <param name="vCount">Vertex Count</param>
	/// <param name="adjacencyMatrix">Adjacent Matrix</param>
	/// <param name="normalized">Normalize Laplacian matrix if true</param>
	/// <param name="weightedSmooth">Boolean to control Laplacian matrix. Implement for "false" first.</param>
	/// <returns>Laplacian matrix: MathNet.Numerics.LinearAlgebra.Single.SparseMatrix</returns>
	public static SparseMatrix BuildLaplacianMatrixFromAdjacentMatrix(
		int vCount, int[,] adjacencyMatrix, bool normalize = true, bool weightedSmooth = false)
	{
		Profiler.BeginSample("BuildLaplacianMatrixFromAdjacentMatrix");
		SparseMatrix lapl = new SparseMatrix(vCount, vCount);
		int maxNeighbors = adjacencyMatrix.GetLength(1);

		for (int vi = 0; vi < vCount; vi++)
		{
			int viDeg = 0;
			for (int j = 0; j < maxNeighbors; j++)
			{
				int vj = adjacencyMatrix[vi, j];
				if (vj < 0)
                {
					break;
                }
				++viDeg;
				lapl.At(vi, vj, -1);
			}

			if (!normalize)
            {
				lapl.At(vi, vi, viDeg);
			}
			else
			{
				for (int j = 0; j < maxNeighbors; j++)
				{
					int vj = adjacencyMatrix[vi, j];
					if (vj < 0)
					{
						break;
					}
					lapl.At(vi, vj, lapl.At(vi, vj) / viDeg);
				}
				lapl.At(vi, vi, 1.0f);
			}

		}

		Profiler.EndSample();
		return lapl;
	}

	/// <summary>
	/// Build smooth matrix from laplacian. (So slow...)
	/// </summary>
	/// <param name="lapl">Laplacian matrix</param>
	/// <param name="smoothLambda">Lambda parameter</param>
	/// <param name="iteration">iteration</param>
	/// <returns>Smooth matrix: MathNet.Numerics.LinearAlgebra.Single.SparseMatrix</returns>
	public static SparseMatrix BuildSmoothMatrixFromLaplacian(SparseMatrix lapl, float smoothLambda, int iteration)
	{
		Profiler.BeginSample("BuildSmoothMatrixFromLaplacian");
		int vCount = lapl.ColumnCount;
		SparseMatrix identity = SparseMatrix.CreateIdentity(vCount);
		//SparseMatrix b = (identity + (smoothLambda / iteration) * lapl);
		//SparseMatrix bt = new SparseMatrix(vCount);
		//b.Transpose(bt);

		//SparseMatrix smooth = SparseMatrix.CreateIdentity(vCount);
		//SparseMatrix smoothNext = new SparseMatrix(vCount); 
		//GpBiCg gpBiCg = new GpBiCg();
		//for (int i = 0; i < iteration; ++i)
		//      {
		//	bt.TrySolveIterative(smooth, smoothNext, gpBiCg);
		//	smooth = smoothNext;
		//      }

		SparseMatrix a = (identity - (smoothLambda / iteration) * lapl);

		SparseMatrix smooth = SparseMatrix.CreateIdentity(vCount);
		SparseMatrix smoothNext = new SparseMatrix(vCount);
		for (int i = 0; i < iteration; ++i)
		{
			smooth.Multiply(a, smoothNext);
			smooth = smoothNext;
		}


		Profiler.EndSample();
		return smooth;
	}
}