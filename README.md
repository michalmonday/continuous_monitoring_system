
# Continuous monitoring system

Components created so far:
* **trace_filter.v** - outputs `drop_instr=HIGH` when instruction isn't a branch/jump/return, which can be used to avoid storing such instructions
* **dram.v** - it's not used yet, but could be used to store any data (e.g. hashes of basic blocks)
* **shift_register_generic.v** - allows to use 32-bit AXI GPIO to control the operational mode of the continuous monitoring system despite requiring more control input bits
* **performance_event_counter.v** - Flute processor has 115 performance counters (of which there are 85 active ones), this module takes bitmap input (where each bit indicates whether the corresponding performance counter should be incremented) and outputs the value of the performance counters (using limited bits, 7-per each counter)  [performance_event_names.txt](./src_pynq/performance_event_names.txt) contains names of each counter, ot was created by the [read_vcd.py](https://github.com/michalmonday/Flute/blob/continuous_monitoring/builds/RV64ACDFIMSUxCHERI_Flute_verilator/vcd/read_vcd.py) script (created because gtkwave doesn't show nicely inputs with large widths, and originally all performance counters occupied 7360 bits in the Flute, 115*64). Performance counters are reset whenever FIFO is enqueued, so only delta of performance counter is sent.
* **cms_ip_wrapper.v** - created solely because it's not possible to use SystemVerilog as a module in Vivado block design (even after packaging IP the top file of IP can't be written in SystemVerilog), we'll have to update this file every time we change inputs/outputs of the `continuous_monitoring_system.sv`, it's a pain but seems necessary.

###Selected performance events mapping:
https://github.com/michalmonday/Flute/blob/continuous_monitoring/builds/RV64ACDFIMSUxCHERI_Flute_verilator/vcd/performance_event_names_selected.csv

