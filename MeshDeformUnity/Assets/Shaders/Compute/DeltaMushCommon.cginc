#ifndef DELTA_MUSH_COMMON_INCLUDED
#define DELTA_MUSH_COMMON_INCLUDED

#define ZERO_VECTOR_3 float3(0, 0, 0)
#define ZERO_VECTOR_4 float4(0, 0, 0, 0)
#define ZERO_MATRIX_3 float3x3(0, 0, 0, 0, 0, 0, 0, 0, 0)
#define ZERO_MATRIX_4 float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#define IDENTITY_MATRIX_4 float4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)

#define QUATERNION_IDENTITY float4(0, 0, 0, 1)

#define PI 3.14159265359f

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

struct CompressedPpp
{
    float m0;
    float2 m1;
    float3 m2;
};

struct OmegaLastColumnStructWithIndex
{
    float3 X;
    float omega;
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

float3x3 CompressedPppToMatrix(CompressedPpp P_pp_comp)
{
    return float3x3(
		P_pp_comp.m0, P_pp_comp.m1.x, P_pp_comp.m2.x,
		P_pp_comp.m1.x, P_pp_comp.m1.y, P_pp_comp.m2.y,
		P_pp_comp.m2.x, P_pp_comp.m2.y, P_pp_comp.m2.z);
}

CompressedPpp MatrixToCompressedPpp(float3x3 P_pp)
{
    CompressedPpp P_pp_comp;
    P_pp_comp.m0 = P_pp[0][0];
    P_pp_comp.m1.x = P_pp[0][1];
    P_pp_comp.m1.y = P_pp[1][1];
    P_pp_comp.m2.x = P_pp[0][2];
    P_pp_comp.m2.y = P_pp[1][2];
    P_pp_comp.m2.z = P_pp[2][2];
    return P_pp_comp;
}

OmegaLastColumnStructWithIndex VectorToOmegaLastColumnStructWithIndex(float3 X, float omega, int index)
{
    OmegaLastColumnStructWithIndex res;
    res.X = X;
    res.omega = omega;
    res.boneIndex = index;
    return res;
}

float4 q_look_at(float3 forward, float3 up)
{
    float3 right = normalize(cross(forward, up));
    up = normalize(cross(forward, right));

    float m00 = right.x;
    float m01 = right.y;
    float m02 = right.z;
    float m10 = up.x;
    float m11 = up.y;
    float m12 = up.z;
    float m20 = forward.x;
    float m21 = forward.y;
    float m22 = forward.z;

    float num8 = (m00 + m11) + m22;
    float4 q = QUATERNION_IDENTITY;
    if (num8 > 0.0)
    {
        float num = sqrt(num8 + 1.0);
        q.w = num * 0.5;
        num = 0.5 / num;
        q.x = (m12 - m21) * num;
        q.y = (m20 - m02) * num;
        q.z = (m01 - m10) * num;
        return q;
    }

    if ((m00 >= m11) && (m00 >= m22))
    {
        float num7 = sqrt(((1.0 + m00) - m11) - m22);
        float num4 = 0.5 / num7;
        q.x = 0.5 * num7;
        q.y = (m01 + m10) * num4;
        q.z = (m02 + m20) * num4;
        q.w = (m12 - m21) * num4;
        return q;
    }

    if (m11 > m22)
    {
        float num6 = sqrt(((1.0 + m11) - m00) - m22);
        float num3 = 0.5 / num6;
        q.x = (m10 + m01) * num3;
        q.y = 0.5 * num6;
        q.z = (m21 + m12) * num3;
        q.w = (m20 - m02) * num3;
        return q;
    }

    float num5 = sqrt(((1.0 + m22) - m00) - m11);
    float num2 = 0.5 / num5;
    q.x = (m20 + m02) * num2;
    q.y = (m21 + m12) * num2;
    q.z = 0.5 * num5;
    q.w = (m01 - m10) * num2;
    return q;
}

float4 vector_to_quaternion(float3 vec)
{
    //float3 up = normalize(cross(vec, float3(0, 1, 0)));
    return normalize(q_look_at(vec, float3(0, 1, 0)));
}

float3x3 quaternion_to_matrix(float4 quat)
{
    float3x3 m = ZERO_MATRIX_3;

    float x = quat.x, y = quat.y, z = quat.z, w = quat.w;
    float x2 = x + x, y2 = y + y, z2 = z + z;
    float xx = x * x2, xy = x * y2, xz = x * z2;
    float yy = y * y2, yz = y * z2, zz = z * z2;
    float wx = w * x2, wy = w * y2, wz = w * z2;

    m[0][0] = 1.0 - (yy + zz);
    m[0][1] = xy - wz;
    m[0][2] = xz + wy;

    m[1][0] = xy + wz;
    m[1][1] = 1.0 - (xx + zz);
    m[1][2] = yz - wx;

    m[2][0] = xz - wy;
    m[2][1] = yz + wx;
    m[2][2] = 1.0 - (xx + yy);

    return m;
}

IndexWeightPair FloatToIndexWeightPair(float omega, int index)
{
    IndexWeightPair res;
    res.weight = omega;
    res.index = index;
    return res;
}

#endif // DELTA_MUSH_COMMON_INCLUDED