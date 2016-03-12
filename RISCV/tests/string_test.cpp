#include <stdint.h>
#include <stdio.h>

void test_loop2(char alphabet0[26],char alphabet1[26]){
	
	char* str_temp;
	int32_t i = 0;
	
	if(alphabet0[0] != 'a'){

		for(i=0;i<26;i++){

			alphabet0[i] = 'a'+i;

		}

	}
	i = 25;
	while(i>=0){
		
		if(alphabet1[i] != alphabet0[25-i]){
			alphabet1[i] = alphabet0[25-i];
			
		}
		else{
			alphabet1[i] = alphabet0[25-i] - 32u;
		}
		i--;
	}
}