with open("riscv-example-cheri.mem", "r") as file:
    instruction_list = [line.rstrip() for line in file]

number_of_instructions = len(instruction_list)

binary_list = []
for instruction in instruction_list:
    binary_list.append(format(int(instruction, 16),'0>32b'))

number_of_branches = 0
branch_instruction_address = 0
branch_instruction_address_list = []
branch_instruction_list = []

for word in binary_list:
    if word[25:] == '1100011':
        number_of_branches += 1
        branch_instruction_address_list.append(hex(branch_instruction_address))
        branch_instruction_list.append(instruction_list[branch_instruction_address])
    branch_instruction_address += 1

with open('program_statistics.txt', 'w') as file:
    file.write('Number of RISC-V instructions: ' + str(len(instruction_list)) + '\n')
    file.write('Number of branch instructions: ' + str(number_of_branches) + '\n')
    file.write('Addresses of branch instructions: ' + str(branch_instruction_address_list) + '\n')
    file.write('Corresponding branch instructions: ' + str(branch_instruction_list) + '\n')
    