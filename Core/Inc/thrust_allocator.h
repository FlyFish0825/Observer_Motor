#ifndef THRUST_ALLOCATOR_H
#define THRUST_ALLOCATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define THRUST_ALLOCATOR_THRUSTER_COUNT 8U

/*
 * Fixed 8-thruster layout.
 * Thruster positions, directions and Tmax = 500 N are hard-coded in
 * thrust_allocator.c.
 *
 * Call ONCE after MCU/peripheral initialization:
 *     if (!ThrustAllocator_Init()) { Error_Handler(); }
 *
 * Return:
 *   1 = initialization successful
 *   0 = initialization failed
 */
int ThrustAllocator_Init(void);

/*
 * Allocate desired body wrench to fan1...fan8.
 *
 * Input order:
 *   Fx, Fy, Fz [N]
 *   Mx, My, Mz [N*m]
 *
 * Output:
 *   thrust_out[0] -> fan1 [N]
 *   ...
 *   thrust_out[7] -> fan8 [N]
 *
 * Return:
 *   1 = solution available
 *   0 = allocator not initialized / invalid input / solver failed
 */
int ThrustAllocator_Solve(float Fx,
                          float Fy,
                          float Fz,
                          float Mx,
                          float My,
                          float Mz,
                          float thrust_out[THRUST_ALLOCATOR_THRUSTER_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* THRUST_ALLOCATOR_H */
