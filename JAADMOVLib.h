/* 
 * File:   JAADMOVLib.h
 * Author: Ashvin
 *
 * Created on July 22, 2019, 4:58 PM
 */

#ifndef JAADMOVLib_H
#define	JAADMOVLib_H

void initTurn(int degrees);

int turn(void);

char isTurnFinished(void);

void initFwd(int distance);

int fwd(void);

char isFwdFinished(void);

#endif	/* JAADSM_H */