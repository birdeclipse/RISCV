/* some User specified files */

#ifndef TRACE

#define TRACE

#endif

#include <iostream>       // std::cout
#include <queue>          // std::queue
#include "Vcore.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
#include "stdlib.h"
#include "time.h"
#include "stdint.h"


/*Number of simulation cyles*/
#define NUM_CYCLE ((vluint64_t)700)
/*Half_Period (in ps) of a clock 100.000MHZ clock*/
#define HALF_PER_PS ((vluint64_t)5000)
/*Number of Reset Cycles*/
#define NUM_RESET_CYCLE ((vluint64_t)64)

#define byte_mask_0 			0XFFFFFFFFFFFFFF00
#define byte_mask_1 			0XFFFFFFFFFFFF00FF
#define byte_mask_2 			0XFFFFFFFFFF00FFFF
#define byte_mask_3 			0XFFFFFFFF00FFFFFF
#define byte_mask_4 			0XFFFFFF00FFFFFFFF
#define byte_mask_5 			0XFFFF00FFFFFFFFFF
#define byte_mask_6 			0XFF00FFFFFFFFFFFF
#define byte_mask_7 			0X00FFFFFFFFFFFFFF

#define byte_enable_0 			0x00000000000000FF
#define byte_enable_1 			0x000000000000FF00
#define byte_enable_2 			0x0000000000FF0000
#define byte_enable_3 			0x00000000FF000000
#define byte_enable_4 			0x000000FF00000000
#define byte_enable_5 			0x0000FF0000000000
#define byte_enable_6 			0x00FF000000000000
#define byte_enable_7 			0xFF00000000000000


#define half_word_mask_0		0XFFFFFFFFFFFF0000
#define half_word_mask_1		0XFFFFFFFF0000FFFF
#define half_word_mask_2		0XFFFF0000FFFFFFFF
#define half_word_mask_3		0X0000FFFFFFFFFFFF

#define half_word_enable_0 		0x000000000000FFFF
#define half_word_enable_1 		0x00000000FFFF0000
#define half_word_enable_2 		0x0000FFFF00000000
#define half_word_enable_3 		0xFFFF000000000000

#define Word_mask_0 			0XFFFFFFFF00000000
#define Word_mask_1				0X00000000FFFFFFFF

#define word_enable_0 			0X00000000FFFFFFFF
#define word_enable_1 			0XFFFFFFFF00000000

struct Dcache_req{

	vluint8_t  delay_time;
	vluint64_t dcache_req_addr;
	vluint8_t  dcache_req_rd;
	vluint8_t  dcache_req_op;
};



vluint64_t global_time 	= 0;
VerilatedVcdC* tfp 		= 0;

double sc_time_stamp () {
    return global_time;
}



uint64_t ram[4096];
const unsigned int rom [4][8]	= {

	{0X00054703,0X06100793,0X04f70a63,0X00050713,0X06100793,0X07b00693,0X00f70023,0X0017879b},

	{0X0ff7f793,0X00170713,0Xfed798e3,0X0300006f,0X0007c703,0X00054683,0X00d70663,0X00d78023},

	{0X00c0006f,0Xfe07071b,0X00e78023,0Xfff78793,0X00150513,0Xfcb79ee3,0X00008067,0X01958793},

	{0Xfff58593,0Xfcdff06f,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000}

};



std::queue<Dcache_req> myqueue;

void advance_half_clock(Vcore *top) {
#ifdef TRACE
  tfp->dump(global_time);
#endif

  top->eval();
  top->clk = !top->clk;
  top->eval();

  global_time++;
  if (Verilated::gotFinish())  
    exit(0);
}

void advance_clock(Vcore *top, int nclocks=1){

  for( int i=0;i<nclocks;i++) {
    for (int clk=0; clk<2; clk++) {
      advance_half_clock(top);
    }
  }
}

void icache_handle(Vcore *top);
void mem_operation(Vcore *top);
void mem_write(Vcore *top);
void mem_read(Vcore *top);



int main(int argc, char **argv, char **env) {

	/*User Variables begins here*/
	vluint64_t half_cycyle_count;
	uint16_t i;
	/*User Variables ends here*/
	Verilated::commandArgs(argc, argv);
	/* init top verilog instance HERE!!!*/
	Vcore* top = new Vcore;
	/* init top verilog instance ENDS!!!*/
#ifdef TRACE

	// init trace dump
	Verilated::traceEverOn(true);
	tfp = new VerilatedVcdC;
	top->trace(tfp, 99);
	tfp->spTrace()->set_time_resolution ("1 ns");
	tfp->open("output.vcd");
#endif
	// initialize simulation inputs

	top->clk   = 1;
	top->reset = 1;
	srand(time(0));
	for(i=0;i<8;i++){
		top->icache_ack_data[i] = 0;
	}
	for(i=0;i<4096;i++){
		
		ram[i] = 0;
	}
	top->icache_ack_data_valid = 0;
	top->icache_req_addr_retry = 0;
	top->dcache_req_retry = 0;
	top->dcache_ack_data = 0;
	top->dcache_ack_rd = 0;
	top->dcache_ack_valid = 0;
	
	for(half_cycyle_count=0;half_cycyle_count<(NUM_CYCLE*2);half_cycyle_count++){
		advance_half_clock(top);
		top->reset = (half_cycyle_count<(NUM_RESET_CYCLE*2)?1:0);
		if(half_cycyle_count>((NUM_RESET_CYCLE+1)*2)){
				
			icache_handle(top);
			mem_operation(top);
			if((top->debug_dest_valid == 1)&(top->debug_dest == 2034)&(top->debug_dest_rd == 10)&(top->debug_dest_long == 1)){
				printf("\n Program executed succesully the expected output value is %ld\n",top->debug_dest);
				printf("\n finished!! at %ld us\n",half_cycyle_count/2*10);
				printf("\n Author: YUXUN QIU \n");
				printf("\n---------------------------------------------------------\n");
				printf("\n---------------------------------------------------------\n");
				half_cycyle_count = (NUM_CYCLE*2) - 16;
			}
		}
       advance_half_clock(top);
	}

	top->final();

#ifdef TRACE

	tfp->close();

#endif

	exit(0);

}


void icache_handle(Vcore *top){
	
	int i;
	static int delay_counter =0;
	static int icache_addr =0;
	
	if((top->icache_req_addr_valid) == 0){
		delay_counter = rand() % 20 + 1;	/*reset the delay counter*/
		top->icache_ack_data_valid = 0;
		for(i=0;i<8;i++){
			top->icache_ack_data[i] = 0;
		}
		return ;
	}	
	else{
		if(delay_counter == 0){
			icache_addr = (top->icache_req_addr)>>5;
			top->icache_ack_data_valid = 1;
			for(i=0;i<8;i++){
				top->icache_ack_data[i] = rom [icache_addr][i];
			}
			printf("testbench is sending icache data to the core!\n");
			return ;
		}
		else {
			delay_counter-- ;
			top->icache_ack_data_valid = 0;
			return ;
		}
	}
}

void mem_operation(Vcore *top){

	mem_write(top);/*check if there is an write operation*/
	mem_read(top);/*check if therr is an pending read operation*/
	
	return;
}

void mem_write(Vcore* top){

	Dcache_req pending_req;	
	vluint64_t temp;
	vluint64_t mem_data;	

	uint64_t write_mask;
	uint64_t write_enable;
	

	vluint64_t mem_address = (top->dcache_req_addr) >> 3;
	vluint8_t  mem_op = (top->dcache_req_op);
	vluint8_t  mem_rd = (top->dcache_req_rd);
	vluint8_t LOW_3_BIT = (top->dcache_req_addr) & (0x0000000000000007);
	
	if ((top->dcache_req_valid) == 1) {
		switch (top->dcache_req_op) {
			case 0:case 1:case 2:case 3:case 4:case 5:case 6: {
				pending_req.dcache_req_addr = (top->dcache_req_addr);
				pending_req.dcache_req_op = mem_op;
				pending_req.dcache_req_rd = mem_rd;
				pending_req.delay_time = rand()%20 +1;
				myqueue.push(pending_req);
				printf("testbench: push request to fifo!!!\n");
				break;
			}
	
			case 8: {
				switch(LOW_3_BIT){
					case 0:{
						write_mask = byte_mask_0;
						write_enable = byte_enable_0;
						temp = (top->dcache_req_data);
						break;
					}
					case 1:{
						write_mask = byte_mask_1;
						write_enable = byte_enable_1;
						temp = (top->dcache_req_data)<<8;
						break;
					}
					case 2:{
						write_mask = byte_mask_2;
						write_enable = byte_enable_2;
						temp = (top->dcache_req_data)<<16;
						break;
					}
					case 3:{
						write_mask = byte_mask_3;
						write_enable = byte_enable_3;
						temp = (top->dcache_req_data)<<24;
						break;
					}
					case 4:{
						write_mask = byte_mask_4;
						write_enable = byte_enable_4;
						temp = (top->dcache_req_data)<<32;
						break;
					}
					case 5:{
						write_mask = byte_mask_5;
						write_enable = byte_enable_5;
						temp = (top->dcache_req_data)<<40;
						break;
					}
					case 6:{
						write_mask = byte_mask_6;
						write_enable = byte_enable_6;
						temp = (top->dcache_req_data)<<48;
						break;
					}
					case 7:{
						write_mask = byte_mask_7;
						write_enable = byte_enable_7;
						temp = (top->dcache_req_data)<<56;
						break;
					}
					default: break;
				}
				temp = temp & write_enable;
				mem_data = ram[mem_address] & write_mask;
				ram[mem_address] = temp | mem_data;
				printf("SB: WRITE DATA %lX\n",(temp | mem_data));
				break;
			}//SB
			
			case 9: {
				switch(LOW_3_BIT){
					case 0:{
						write_mask = half_word_mask_0;
						write_enable = half_word_enable_0;
						temp = (top->dcache_req_data);
						break;
					}
					case 2:{
						write_mask = half_word_mask_1;
						write_enable = half_word_enable_1;
						temp = (top->dcache_req_data)<<16;
						break;
					}
					case 4:{
						write_mask = half_word_mask_2;
						write_enable = half_word_enable_2;					
						temp = (top->dcache_req_data)<<32;
						break;
					}
					case 6:{
						write_mask = half_word_mask_3;
						write_enable = half_word_enable_3;
						temp = (top->dcache_req_data)<<48;
						break;
					}
					default: printf("unaligned ram access!!! illegal!!!\n");
				}
				temp = temp & write_enable;
				mem_data = ram[mem_address] & write_mask;
				ram[mem_address] = temp | mem_data;
				printf("SH: WRITE DATA %lX\n",(temp | mem_data));
				break;
			}	//SH
			
			case 10: {
				switch(LOW_3_BIT){
					case 0:{
						write_mask = Word_mask_0;
						write_enable = word_enable_0;
						temp = (top->dcache_req_data);
						break;
					}
					case 4:{
						write_mask = Word_mask_1;
						write_enable = word_enable_1;
						temp = (top->dcache_req_data)<<32;
						break;
					}
					default: printf("unaligned ram access!!! illegal!!!\n");
				}			
				temp = temp & write_enable;
				mem_data = ram[mem_address] & write_mask;
				ram[mem_address] = temp | mem_data;
				printf("SW: WRITE DATA %lX\n",(temp | mem_data));
				break;
			}	//SW
			
			case 11: {
				ram[mem_address] = (top->dcache_req_data);
				printf("SD: WRITE DATA %lX \n",(top->dcache_req_data));
				break;
			}	//SD
			
			default: printf("undefined OP code!!!! illegal!!!\n");
		}
	}
	return;
}

void mem_read(Vcore *top){
	
	int i;
	int size;

	uint8_t LOW_3_BIT;	
	int8_t  byte_signed;	
	int16_t half_word_signed;	
	int32_t word_signed;

	uint64_t byte_signed_temp;
	uint64_t half_word_signed_temp;
	uint64_t word_signed_temp;

	uint64_t byte_unsigned;
	uint64_t half_word_unsigned;	
	uint64_t word_unsigned;
	
	uint64_t read_enable;

	if((top->dcache_ack_valid) == 1){
		
		(top->dcache_ack_valid) = 0;
		(top->dcache_ack_rd) = 0;
		(top->dcache_ack_data) = 0;
		
	}
	else if(!myqueue.empty()){
		size = myqueue.size();
		for(i=0;i<size;i++){		
			if (myqueue.front().delay_time == 0){
				LOW_3_BIT = (myqueue.front().dcache_req_addr) & (0x0000000000000007);
				switch (myqueue.front().dcache_req_op) {
					case 0: {
						switch(LOW_3_BIT){
							case 0:{								
								read_enable = byte_enable_0;
								byte_signed_temp = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_signed = (uint8_t) byte_signed_temp;
								break;
							}
							case 1:{
								read_enable = byte_enable_1;
								byte_signed_temp = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_signed = (uint8_t)(byte_signed_temp>>8);
								break;
							}
							case 2:{
								read_enable = byte_enable_2;
								byte_signed_temp = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_signed = (uint8_t)(byte_signed_temp>>16);								
								break;
							}
							case 3:{
								read_enable = byte_enable_3;
								byte_signed_temp = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_signed = (uint8_t)(byte_signed_temp>>24);								
								break;
							}
							case 4:{
								read_enable = byte_enable_4;
								byte_signed_temp = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_signed = (uint8_t)(byte_signed_temp>>32);								
								break;
							}
							case 5:{
								read_enable = byte_enable_5;
								byte_signed_temp = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_signed = (uint8_t)(byte_signed_temp>>40);								
								break;
							}
							case 6:{
								read_enable = byte_enable_6;
								byte_signed_temp = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_signed = (uint8_t)(byte_signed_temp>>48);								
								break;
							}
							case 7:{
								read_enable = byte_enable_7;
								byte_signed_temp = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_signed = (uint8_t)(byte_signed_temp>>56);								
								break;
							}
							default: break;
						}
						(top->dcache_ack_valid) = 1;
						(top->dcache_ack_rd) = myqueue.front().dcache_req_rd;
						(top->dcache_ack_data) = (int64_t)byte_signed;
				
						break;
					}
					case 1: {
						
						switch(LOW_3_BIT){
							case 0:{
								read_enable = half_word_enable_0;
								half_word_signed_temp = (uint64_t)ram[(myqueue.front().dcache_req_addr)>>3] & read_enable;
								half_word_signed = (uint16_t) half_word_signed_temp ;
								break;
							}

							case 2:{
								read_enable = half_word_enable_1;
								half_word_signed_temp = (uint64_t)ram[(myqueue.front().dcache_req_addr)>>3] & read_enable;
								half_word_signed = (uint16_t)(half_word_signed_temp >> 16);
								break;
							}

							case 4:{
								read_enable = half_word_enable_2;
								half_word_signed_temp = (uint64_t)ram[(myqueue.front().dcache_req_addr)>>3] & read_enable;
								half_word_signed = (uint16_t)(half_word_signed_temp >> 32);
								break;
							}

							case 6:{
								read_enable = half_word_enable_3;
								half_word_signed_temp = (uint64_t)ram[(myqueue.front().dcache_req_addr)>>3] & read_enable;
								half_word_signed = (uint16_t)(half_word_signed_temp >> 48);
								break;
							}
							default: break;
						}
						(top->dcache_ack_valid) = 1;
						(top->dcache_ack_rd) = myqueue.front().dcache_req_rd;
						(top->dcache_ack_data) = (int64_t)half_word_signed;
						break;
					}
					
					
					case 2: {	
						switch(LOW_3_BIT){
							case 0:{
								read_enable = word_enable_0;
								word_signed_temp = (uint32_t)ram[(myqueue.front().dcache_req_addr)>>3] & read_enable;
								word_signed = (uint32_t) word_signed_temp;
								break;
							}

							case 4:{
								read_enable = word_enable_1;
								word_signed_temp = (uint32_t)ram[(myqueue.front().dcache_req_addr)>>3] & read_enable;
								word_signed = (uint32_t) (word_signed_temp >> 32);
								break;
							}
							default: break;
						}
						(top->dcache_ack_valid) = 1;
						(top->dcache_ack_rd) = myqueue.front().dcache_req_rd;
						(top->dcache_ack_data) = (int64_t)word_signed;
						break;
					}
					
					case 3: {
						(top->dcache_ack_valid) = 1;
						(top->dcache_ack_rd) = myqueue.front().dcache_req_rd;
						(top->dcache_ack_data) = ram[(myqueue.front().dcache_req_addr)>>3];
						break;
					}

					case 4: {
						switch(LOW_3_BIT){
							case 0:{
								read_enable = byte_enable_0;
								byte_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								break;
							}
							case 1:{
								read_enable = byte_enable_1;
								byte_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_unsigned = byte_unsigned >> 8;
								break;
							}
							case 2:{
								read_enable = byte_enable_2;
								byte_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								byte_unsigned = byte_unsigned >> 16;
								break;
							}
							case 3:{
								read_enable = byte_enable_3;
								byte_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);	
								byte_unsigned = byte_unsigned >> 24;
								break;
							}
							case 4:{
								read_enable = byte_enable_4;
								byte_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);	
								byte_unsigned = byte_unsigned >> 32;
								break;
							}
							case 5:{
								read_enable = byte_enable_5;
								byte_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);	
								byte_unsigned = byte_unsigned >> 40;
								break;
							}
							case 6:{
								read_enable = byte_enable_6;
								byte_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);	
								byte_unsigned = byte_unsigned >> 48;
								break;
							}
							case 7:{
								read_enable = byte_enable_7;
								byte_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);	
								byte_unsigned = byte_unsigned >> 56;
								break;
							}
							default: break;
						}						
						(top->dcache_ack_valid) = 1;
						(top->dcache_ack_rd) = myqueue.front().dcache_req_rd;

						(top->dcache_ack_data) = (uint64_t)byte_unsigned;
						break;
					}
					case 5: {
						switch(LOW_3_BIT){
							case 0:{
								read_enable = half_word_enable_0;
								half_word_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);								
								break;
							}

							case 2:{
								read_enable = half_word_enable_1;
								half_word_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								half_word_unsigned = half_word_unsigned >> 16;
								break;
							}

							case 4:{
								read_enable = half_word_enable_2;
								half_word_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								half_word_unsigned = half_word_unsigned >> 32;								
								break;
							}

							case 6:{
								read_enable = half_word_enable_3;
								half_word_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								half_word_unsigned = half_word_unsigned >> 48;									
								break;
							}
							default: break;
						}
						(top->dcache_ack_valid) = 1;
						(top->dcache_ack_rd) = myqueue.front().dcache_req_rd;

						(top->dcache_ack_data) = (uint64_t)half_word_unsigned;

						break;
					}
					case 6: {
						switch(LOW_3_BIT){
							case 0:{
								read_enable = word_enable_0;
								word_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);						
								break;
							}

							case 4:{
								read_enable = word_enable_1;
								word_unsigned = (uint64_t)(ram[(myqueue.front().dcache_req_addr)>>3] & read_enable);
								word_unsigned = word_unsigned >>32 ;
								break;
							}
							default: break;
						}
						(top->dcache_ack_valid) = 1;
						(top->dcache_ack_rd) = myqueue.front().dcache_req_rd;

						(top->dcache_ack_data) = (uint64_t)word_unsigned;
						break;
					}
					default: break;
				}
				myqueue.pop();

				return;
			}
			else {
				myqueue.front().delay_time--;
				myqueue.push(myqueue.front());
				myqueue.pop();
			}			
		}
	}
	return;
	
}


