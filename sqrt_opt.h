#ifndef SQRT_OPT_H
#define SQRT_OPT_H

typedef float v4sf __attribute__ ((vector_size (16)));
typedef int v4si __attribute__ ((vector_size (16)));


template <size_t LOOPS = 2>
float sqrt1(float * a) {
  float root;
  // from here
  root = 0; // to avoid a warning, delete this in your code
  int * ai = reinterpret_cast<int *>(a);
  int * initial = reinterpret_cast<int *>(&root);
  *initial = (1<<29) + (*ai >> 1) - (1 << 22) - 0x4C000;
  for(size_t i=0; i<LOOPS; i++) {
	root = 0.5 * (root + *a / root);
  }
  // to here
  return root;
}

template <size_t LOOPS = 2>
void sqrt2(float * __restrict__ a, float * __restrict__ root) {
  // from here
  int * init;
  for (unsigned int i = 0; i < 4; i++) {
    init = reinterpret_cast<int *>(a + i);
    int * initial = reinterpret_cast<int *>(&root[i]);
    *initial = (1 << 29) + (*init >> 1) - (1 << 22) - 0x4C000;
  }
  
  for(size_t i=0; i<LOOPS; i++) {
	root[0] = 0.5 * (root[0] + a[0] / root[0]);
	root[1] = 0.5 * (root[1] + a[1] / root[1]);
	root[2] = 0.5 * (root[2] + a[2] / root[2]);
	root[3] = 0.5 * (root[3] + a[3] / root[3]);
  }
  // to here
}


template <size_t LOOPS = 2>
void v4sf_sqrt(v4sf *  __restrict__  a, v4sf *  __restrict__  root) {
  // from here
  int * init;
  float *aPtr = (float *)a;
  float *rootPtr = (float *)root;
  for (unsigned int i = 0; i < 4; i++) {
    init = reinterpret_cast<int *>(aPtr + i);
    int * initial = reinterpret_cast<int *>(&rootPtr[i]);
    *initial = (1 << 29) + (*init >> 1) - (1 << 22) - 0x4C000;
  }
  for (unsigned int i = 0; i < LOOPS; i++) {
    *root = 0.5 * (*root + (*a / *root));
  }
  // to here
}


// wrapper für v4sf_sqrt
template <size_t LOOPS = 2>
void sqrt3(float *  __restrict__  a, float *  __restrict__  root) {
  v4sf *as =  reinterpret_cast<v4sf *>(a);
  v4sf_sqrt<LOOPS>(as, reinterpret_cast<v4sf *>(root) );
}

#endif

