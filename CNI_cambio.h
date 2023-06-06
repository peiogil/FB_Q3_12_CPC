/* 
 * File:   CNI_cambio.h
 * Author: peio.gil
 *
 * Created on 5 de junio de 2023, 20:47
 */

#ifndef CNI_CAMBIO_H
#define	CNI_CAMBIO_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif
typedef enum 
{
    
    CNI_EST_ON1=0,
    CNI_EST_ON2,
    CNI_EST_OFF1,
    CNI_EST_OFF2
            
}CNI_CHANGE;

static CNI_CHANGE CNI_EST= CNI_EST_OFF1;

#endif	/* CNI_CAMBIO_H */

