#ifndef DELTA_MUSH_COMMON_INCLUDED
#define DELTA_MUSH_COMMON_INCLUDED

#define ZERO_VECTOR_3 float3(0, 0, 0)
#define ZERO_VECTOR_4 float4(0, 0, 0, 0)
#define ZERO_MATRIX_3 float3x3(0, 0, 0, 0, 0, 0, 0, 0, 0)
#define ZERO_MATRIX_4 float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#define IDENTITY_MATRIX_4 float4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)

#define MAX_OMEGA_COUNT 16

struct IndexWeightPair
{
	int index;
	float weight;
};

struct BoneWeights
{
	float4 weights;
	int4 indices;
};

struct OmegaStructWithIndex
{
	float m0;
	float2 m1;
	float3 m2;
	float4 m3;
	int boneIndex;
};

float4x4 OmegaStructWithIndexToMatrix(OmegaStructWithIndex oswi)
{
	return float4x4(
		oswi.m0,   oswi.m1.x, oswi.m2.x, oswi.m3.x,
		oswi.m1.x, oswi.m1.y, oswi.m2.y, oswi.m3.y,
		oswi.m2.x, oswi.m2.y, oswi.m2.z, oswi.m3.z,
		oswi.m3.x, oswi.m3.y, oswi.m3.z, oswi.m3.w);
}

OmegaStructWithIndex MatrixToOmegaStructWithIndex(float4x4 mat, int index)
{
	OmegaStructWithIndex oswi;
	oswi.m0 = mat[0][0];
	oswi.m1.x = mat[0][1];
	oswi.m1.y = mat[1][1];
	oswi.m2.x = mat[0][2];
	oswi.m2.y = mat[1][2];
	oswi.m2.z = mat[2][2];
	oswi.m3.x = mat[0][3];
	oswi.m3.y = mat[1][3];
	oswi.m3.z = mat[2][3];
	oswi.m3.w = mat[3][3];
	oswi.boneIndex = index;
	return oswi;
}

#endif // DELTA_MUSH_COMMON_INCLUDED