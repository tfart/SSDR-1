/**** Decompose.h - Basic declarations ****/
#ifndef _H_Decompose
#define _H_Decompose


enum QuatPart {X, Y, Z, W};

typedef struct {float x, y, z, w;} Quat; /* Quaternion */
typedef Quat HVect; /* Homogeneous 3D vector */
typedef float HMatrix[4][4]; /* Right-handed, for column vectors */
typedef struct {
    HVect t;	/* Translation components */
    Quat  q;	/* Essential rotation	  */
    Quat  u;	/* Stretch rotation	  */
    HVect k;	/* Stretch factors	  */
    float f;	/* Sign of determinant	  */
} AffineParts;



#ifdef __cplusplus
extern "C" {
#endif
float polar_decomp(HMatrix M, HMatrix Q, HMatrix S);
HVect spect_decomp(HMatrix S, HMatrix U);
Quat snuggle(Quat q, HVect *k);
void decomp_affine(HMatrix A, AffineParts *parts);
void invert_affine(AffineParts *parts, AffineParts *inverse);

#ifdef __cplusplus
}
#endif

#endif
