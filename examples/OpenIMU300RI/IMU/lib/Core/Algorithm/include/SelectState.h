/* 
 * File:   SelectState.h
 * Author: joemotyka
 *
 * Created on May 6, 2016, 11:37 PM
 */

#ifndef SELECTSTATE_H
#define SELECTSTATE_H

void StabilizeSystem(void);
void InitializeAttitude(void);
void HG_To_LG_Transition_Test(void);
void LG_To_INS_Transition_Test(void);
void INS_To_AHRS_Transition_Test(void);

void DynamicMotion(void);

#endif /* SELECTSTATE_H */
