#ifndef	_DEBUG_H__
#define	_DEBUG_H__

#include <stm32f10x_type.h>
/*���������_DEBUG_���������ڵ��Ե�USART�����һϵ�к���*/	
extern  void PrintS(u8 *str);
extern  void PrintHex(u8 ch);

#ifdef _DEBUG_

#define	printf(a)	PrintS(a)
#define	printhex(a) PrintHex(a)
#else
#define	printf(a)
#define	printhex(a)
#endif

#endif
