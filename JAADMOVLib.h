/* 
 * File:   JAADMOVLib.h
 * Author: Ashvin
 *
 * Created on July 22, 2019, 4:58 PM
 */

#ifndef JAADMOVLib_H
#define	JAADMOVLib_H

void MOV_initTurn(int degrees);

int MOV_updateTurn(void);

char MOV_isTurnFinished(void);

void MOV_initFwd(int distance);

int MOV_updateFwd(void);

char MOV_isFwdFinished(void);

#endif	/* JAADSM_H */