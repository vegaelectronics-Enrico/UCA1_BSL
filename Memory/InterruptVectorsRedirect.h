/*
 * InterruptVectorRedirect.h
 *
 *  Created on: 20/mag/2015
 *      Author: Alberto
 */

#ifndef SHAREDSOURCES_MEMORY_INTERRUPTVECTORSREDIRECT_H_
#define SHAREDSOURCES_MEMORY_INTERRUPTVECTORSREDIRECT_H_

///////////////////////////////////////////////////////////////////////////////

#define PROXY_VECTORS_NUMBER					55
#define INT_VECTOR_START        				( (uint32_t )ISR_redirect )
#define INT_VECTOR_END          				( (uint32_t )&proxy_vectors[PROXY_VECTORS_NUMBER] + 1 )

///////////////////////////////////////////////////////////////////////////////

extern void ISR_redirect(void);
extern const unsigned short int proxy_vectors[PROXY_VECTORS_NUMBER];

///////////////////////////////////////////////////////////////////////////////

#endif /* SHAREDSOURCES_MEMORY_INTERRUPTVECTORSREDIRECT_H_ */
