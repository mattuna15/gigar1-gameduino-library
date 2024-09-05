#define LOAD_ASSETS()  GD.safeload("COBRA.GD2");
#define BACKGROUND_HANDLE 0
#define BACKGROUND_WIDTH 256
#define BACKGROUND_HEIGHT 256
#define BACKGROUND_CELLS 1
#define SUN_HANDLE 1
#define SUN_WIDTH 150
#define SUN_HEIGHT 150
#define SUN_CELLS 1
#define LIGHT_HANDLE 2
#define LIGHT_WIDTH 30
#define LIGHT_HEIGHT 30
#define LIGHT_CELLS 1
#define ASSETS_END 176972UL
static const shape_t BACKGROUND_SHAPE = {0, 256, 256, 0};
static const shape_t SUN_SHAPE = {1, 150, 150, 0};
static const shape_t LIGHT_SHAPE = {2, 30, 30, 0};
static const PROGMEM int8_t COBRA_vertices[] = {
15,0,37, -15,0,37, 0,12,11, -59,-1,-3, 59,-1,-3, -43,7,-19, 43,7,-19, 63,-3,-19, -63,-3,-19, 0,12,-19, -15,-11,-19, 15,-11,-19, -17,3,-19, -3,5,-19, 3,5,-19, 17,3,-19, 17,-5,-19, 3,-7,-19, -3,-7,-19, -17,-5,-19, 0,0,37, 0,0,44, -39,-2,-19, -39,2,-19, -43,0,-19, 39,2,-19, 43,0,-19, 39,-2,-19, 63,-3,-19, -63,-3,-19
};
static const PROGMEM int8_t COBRA_faces[] = {
3,0,115,53,7,0,0,0,0,2,0,1,
3,-36,117,32,28,0,0,0,0,2,1,5,
3,36,117,32,97,0,0,0,0,6,0,2,
3,-29,119,29,(int8_t)136,1,0,0,0,5,1,3,
3,29,119,29,32,6,0,0,0,4,0,6,
3,-14,126,0,16,24,0,0,0,9,2,5,
3,14,126,0,64,40,0,0,0,6,2,9,
3,-56,113,0,0,(int8_t)193,0,0,0,5,3,8,
3,56,113,0,0,4,3,0,0,7,4,6,
7,0,0,-127,0,(int8_t)176,30,0,0,6,9,5,8,10,11,7,
4,-19,-123,24,(int8_t)128,64,48,0,0,8,3,1,10,
4,0,-124,24,2,0,104,0,0,1,0,11,10,
4,19,-123,24,0,2,69,0,0,11,0,4,7,
4,0,0,-126,0,0,(int8_t)128,7,0,16,15,14,17,
4,0,0,-126,0,0,0,120,0,18,13,12,19,
3,0,0,-127,0,0,0,(int8_t)128,3,24,22,23,
3,0,0,-127,0,0,0,0,28,26,25,27,
-1
};
static const PROGMEM uint8_t COBRA_edges[] = {
0,2, 0,1, 1,2, 1,5, 2,5, 0,6, 2,6, 1,3, 3,5, 0,4, 4,6, 2,9, 5,9, 6,9, 3,8, 5,8, 4,7, 6,7, 7,11, 10,11, 8,10, 1,10, 0,11, 14,17, 14,15, 15,16, 16,17, 12,19, 12,13, 13,18, 18,19, 22,23, 22,24, 23,24, 25,27, 25,26, 26,27
};
static const PROGMEM uint8_t shiny[512] = {
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,12,12,12,12,12,12,12,13,13,13,13,13,13,13,14,14,14,14,14,14,15,15,15,15,15,15,16,16,16,16,16,16,17,17,17,17,17,18,18,18,18,18,19,19,19,19,19,20,20,20,20,20,21,21,21,21,22,22,22,22,22,23,23,23,23,24,24,24,24,25,25,25,25,26,26,26,27,27,27,27,28,28,28,29,29,29,29,30,30,30,31,31,31,32,32,32,33,33,33,34,34,34,35,35,35,36,36,36,37,37,37,38,38,39,39,39,40,40,41,41,41,42,42,43,43,43,44,44,45,45,46,46,47,47,47,48,48,49,49,50,50,51,51,52,52,53,53,54,54,55,55,56,56,57,57,58,58,59,60,60,61,61,62,62,63,64,64,65,65,66,67,67,68,69,69,70,70,71,72,72,73,74,74,75,76,76,77,78,79,79,80,81,81,82,83,84,84,85,86,87,88,88,89,90,91,92,92,93,94,95,96,97,97,98,99,100,101,102,103,104,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,130,131,132,133,134,135,136,138,139,140,141,142,143,145,146,147,148,150,151,152,153,155,156,157,159,160,161,163,164,165,167,168,169,171,172,174,175,176,178,179,181,182,184,185,187,188,190,191,193,195,196,198,199,201,203,204,206,208,209,211,213,214,216,218,219,221,223,225,227,228,230,232,234,236,238,239,241,243,245,247,249,251,253,255
};
