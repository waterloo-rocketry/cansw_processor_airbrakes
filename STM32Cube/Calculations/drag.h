#ifndef DRAG_H_
#define DRAG_H_

/**@return drag force acting on rocket
 * @param extension of air brakes, used for adjusting rocket area and
 * iterpolating Ansys sims (0-1)
 * @param velocity used to lookup drag force from Ansys sims
 * @param altitude used to adjust fluid density since Ansys sims were calculated
 * at 1000m
 * */
float interpolate_drag(float extension, float velocity, float altitude);

#endif
