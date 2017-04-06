#define BEAM 32
#define MAX_BIM 7
#define RANGES 20
#define SAFE_SIDE BEAM/5

typedef struct elt {
  int y;
  int width;
  struct elt * next;
} stack_type;


typedef struct {
  int lx;
  int ry;
} intervall;
