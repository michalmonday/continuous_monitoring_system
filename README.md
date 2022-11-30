
# Continuous monitoring system


Components created so far:
* **trace_filter.v** - outputs `drop_instr=HIGH` when instruction isn't a branch/jump/return, which can be used to avoid storing such instructions
* **dram.v** - it's not used yet, but could be used to store any data (e.g. hashes of basic blocks)