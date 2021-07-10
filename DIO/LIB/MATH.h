/*
 * Autonomous.c
 *
 * Created: 6/30/2021 10:47:45 AM
 * Author : Sayed Gomaa
 */

#ifndef MATH_H_
#define MATH_H_

#define SET_BIT(VAR,BITNO) (VAR)|=(1<<(BITNO))

#define CLR_BIT(VAR,BITNO) (VAR)&=~(1<<(BITNO))

#define TOGGLE_BIT(VAR,BITNO) (VAR)^=(1<<(BITNO))

#define GET_BIT(VAR,BITNO) ((VAR)>>(BITNO))&(0x01)

#endif /* MATH_H_ */
