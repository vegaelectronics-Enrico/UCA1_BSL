
#include <Memory/InterruptVectorsRedirect.h>

//redirecting interrupts
#pragma RETAIN (ISR_redirect)
#pragma CODE_SECTION(ISR_redirect,".brintvec")

/*
 * "proxy_vectors" è la sezione originale che contiene gli address a cui saltare per servire le ISR.
 * La sovrascrivo in modo che ogni interrupt punti ad una istruzione di branch (4 byte) contenuta in "brintvec", che a sua volta esgue il salto all'indirizzo contenuto nella tabella corrispondente alla funzione da eseguire
 * (che è generato dal compilatore del programma in esecuzione)
 */



void ISR_redirect(void)
{
asm(" BR &0007C00h ");
asm(" BR &0007C02h ");
asm(" BR &0007C04h ");
asm(" BR &0007C06h ");
asm(" BR &0007C08h ");
asm(" BR &0007C0Ah ");
asm(" BR &0007C0Ch ");
asm(" BR &0007C0Eh ");
asm(" BR &0007C10h ");
asm(" BR &0007C12h ");//10
asm(" BR &0007C14h ");
asm(" BR &0007C16h ");
asm(" BR &0007C18h ");
asm(" BR &0007C1Ah ");
asm(" BR &0007C1Ch ");
asm(" BR &0007C1Eh ");
asm(" BR &0007C20h ");
asm(" BR &0007C22h ");
asm(" BR &0007C24h ");
asm(" BR &0007C26h ");//20
asm(" BR &0007C28h ");
asm(" BR &0007C2Ah ");
asm(" BR &0007C2Ch ");
asm(" BR &0007C2Eh ");
asm(" BR &0007C30h ");
asm(" BR &0007C32h ");
asm(" BR &0007C34h ");
asm(" BR &0007C36h ");
asm(" BR &0007C38h ");
asm(" BR &0007C3Ah ");//30
asm(" BR &0007C3Ch ");
asm(" BR &0007C3Eh ");
asm(" BR &0007C40h ");
asm(" BR &0007C42h ");
asm(" BR &0007C44h ");
asm(" BR &0007C46h ");
asm(" BR &0007C48h ");
asm(" BR &0007C4Ah ");
asm(" BR &0007C4Ch ");
asm(" BR &0007C4Eh ");//40
asm(" BR &0007C50h ");
asm(" BR &0007C52h ");
asm(" BR &0007C54h ");
asm(" BR &0007C56h ");
asm(" BR &0007C58h ");
asm(" BR &0007C5Ah ");
asm(" BR &0007C5Ch ");
asm(" BR &0007C5Eh ");
asm(" BR &0007C60h ");
asm(" BR &0007C62h ");//50
asm(" BR &0007C64h ");
asm(" BR &0007C66h ");
asm(" BR &0007C68h ");
asm(" BR &0007C6Ah ");//54
//asm(" BR &0007C6Ch ");//55 - RESET
}

#pragma RETAIN (proxy_vectors)  //forza a generare output anche se la variabile costante non è mai utilizzata nel codice
#pragma DATA_SECTION(proxy_vectors,".proxy_vectors")
const unsigned short int proxy_vectors[PROXY_VECTORS_NUMBER]={
        0xFE00,
        0xFE00+4,
        0xFE00+8,
        0xFE00+12,
        0xFE00+16,
        0xFE00+20,
        0xFE00+24,
        0xFE00+28,
        0xFE00+32,
        0xFE00+36,//10
        0xFE00+40,
        0xFE00+44,
        0xFE00+48,
        0xFE00+52,
        0xFE00+56,
        0xFE00+60,
        0xFE00+64,
        0xFE00+68,
        0xFE00+72,
        0xFE00+76,//20
        0xFE00+80,
        0xFE00+84,
        0xFE00+88,
        0xFE00+92,
        0xFE00+96,
        0xFE00+100,
        0xFE00+104,
        0xFE00+108,
        0xFE00+112,
        0xFE00+116,//30
        0xFE00+120,
        0xFE00+124,
        0xFE00+128,
        0xFE00+132,
        0xFE00+136,
        0xFE00+140,
        0xFE00+144,
        0xFE00+148,
        0xFE00+152,
        0xFE00+156,//40
        0xFE00+160,
        0xFE00+164,
        0xFE00+168,
        0xFE00+172,
        0xFE00+176,
        0xFE00+180,
        0xFE00+184,
        0xFE00+188,
        0xFE00+192,
        0xFE00+196,//50
        0xFE00+200,
        0xFE00+204,
        0xFE00+208,
        0xFE00+212, //54
        0xFE00+216  //55- RESET
};
