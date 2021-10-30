#ifdef __cplusplus
extern "C" {
#endif

#include "encoder.h"

int en_pos1 = 0;
int en_pos2 = 0;
int en_pos3 = 0;
int en_pos4 = 0;

void count_pos1()
{
		if(digitalRead_FAST(ENCODE1_A) == HIGH){
			if(digitalRead_FAST(ENCODE1_B)== LOW){
				en_pos1++;
			}else {
				en_pos1--;
			}
		} else {
			if(digitalRead_FAST(ENCODE1_B)== HIGH){
				en_pos1++;
			}else {
				en_pos1--;
			}
		}
}
void count_pos2()
{
		if(digitalRead_FAST(ENCODE2_A) == HIGH){
			if(digitalRead_FAST(ENCODE2_B)== LOW){
				en_pos2++;
			}else {
				en_pos2--;
			}
		} else {
			if(digitalRead_FAST(ENCODE2_B)== HIGH){
				en_pos2++;
			}else {
				en_pos2--;
			}
		}
}

void count_pos3()
{
		if(digitalRead_FAST(ENCODE3_A) == HIGH){
			if(digitalRead_FAST(ENCODE3_B)== LOW){
				en_pos3++;
			}else {
				en_pos3--;
			}
		} else {
			if(digitalRead_FAST(ENCODE3_B)== HIGH){
				en_pos3++;
			}else {
				en_pos3--;
			}
		}
}

void count_pos4()
{
		if(digitalRead_FAST(ENCODE4_A) == HIGH){
			if(digitalRead_FAST(ENCODE4_B)== LOW){
				en_pos4++;
			}else {
				en_pos4--;
			}
		} else {
			if(digitalRead_FAST(ENCODE4_B)== HIGH){
				en_pos4++;
			}else {
				en_pos4--;
			}
		}
}

void encoder_init()
{
    pinMode(ENCODE1_A, INPUT_PULLUP);
	  pinMode(ENCODE1_B, INPUT_PULLUP);
	
    pinMode(ENCODE2_A, INPUT_PULLUP);
    pinMode(ENCODE2_B, INPUT_PULLUP);
	
    pinMode(ENCODE3_A, INPUT_PULLUP);
    pinMode(ENCODE3_B, INPUT_PULLUP);
	
    pinMode(ENCODE4_A, INPUT_PULLUP);
    pinMode(ENCODE4_B, INPUT_PULLUP);
	
    attachInterrupt(ENCODE1_A, count_pos1, CHANGE);
    attachInterrupt(ENCODE2_A, count_pos2, CHANGE);
    attachInterrupt(ENCODE3_A, count_pos3, CHANGE);
    attachInterrupt(ENCODE4_A, count_pos4, CHANGE);

}


#ifdef __cplusplus
}
#endif
