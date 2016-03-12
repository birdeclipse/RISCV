
core:
	verilator  --cc --trace --Wall ./rtl/core.v ./rtl/icache.v ./rtl/fetch.v ./rtl/regfile.v ./rtl/decode.v ./rtl/execute.v ./rtl/flop.v ./rtl/sram.v ./rtl/Fluid_Flop.v ./rtl/pending_check.v ./rtl/flop_e.v ./rtl/mem_access.v -exe tests/pipeline_core_test.cpp -CFLAGS -DTRACE=1
	make -C obj_dir/ -f Vcore.mk Vcore



core0:
	verilator  --cc --trace --Wall ./rtl/core.v ./rtl/icache.v ./rtl/fetch.v ./rtl/regfile.v ./rtl/decode.v ./rtl/execute.v ./rtl/flop.v ./rtl/sram.v ./rtl/Fluid_Flop.v ./rtl/pending_check.v ./rtl/flop_e.v ./rtl/mem_access.v -exe tests/core_string_test.cpp -CFLAGS -DTRACE=1
	make -C obj_dir/ -f Vcore.mk Vcore



run: core
	obj_dir/Vcore
	gtkwave output.vcd &


run0: core0
	obj_dir/Vcore
	gtkwave output.vcd &
	
clean:
	rm -rf obj_dir output.vcd


