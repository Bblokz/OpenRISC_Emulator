#load word extend with zero
# Example of how to write tests for load instructions. In order to
# execute a load instruction, a readable memory must be available.
# To do so, we allocate a memory in the assembly program ("A" in
# this case). After assembling the program, the memory location of the
# array must be determined. This can be done using "objdump". In this
# case, the location is 0x11100 (69888 decimal). We initialize the "x1"
# (R1) register with the value 69888. This way, the load instructions in
# the program below will read from valid memory addresses where also
# initialized data is found, which allows the register values to be verified
# when the program terminates.

       .data
       .align 8
       .local  A
A:
       .int 10, 20, 30, 40, 50, 60, 70, 80
       .size   A, .-A
       .text
       .align 4
       .globl  _start
       .type   _start, @function
_start:
       l.lwz r2,0(r1)             # 10
       l.lwz r3,4(r1)             # 20
       l.lwz r4,8(r1)             # 30
       .word  0x40ffccff
       .size   _start, .-_start
