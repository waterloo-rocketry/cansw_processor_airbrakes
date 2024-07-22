#ifndef TRAJECTORY_LIB_H_
#define TRAJECTORY_LIB_H_

#define ROCKET_BURNOUT_MASS 39.564  // kg

/**@return drag force acting on rocket
 * @param extension of air brakes, used for adjusting rocket area and
 * iterpolating Ansys sims (0-1)
 * @param velocity used to lookup drag force from Ansys sims
 * @param altitude used to adjust fluid density since Ansys sims were calculated
 * at 1000m
 * */
float interpolate_drag(float extension, float velocity, float altitude);

/** @return max apogee
 * @param velocity vertical velocity (m/s)
 * @param altitude (m)
 * @param airbrake_ext extension of airbrakes, 0-1
 * @param mass (kg)
 */
float get_max_altitude(float velY, float velX, float altitude,
                       float airbrake_ext, float mass);

#endif