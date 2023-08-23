beq zero, zero, main

prime:
beq a2, a1, one
blt a1, a2, null
rem t0, a1, a2
beq t0, zero, null
addi a2, a2, 1
beq zero, zero, prime

one:
addi a1, zero, 1
jalr zero, ra, 0

null:
addi a1, zero, 0
jalr zero, ra, 0

done:
beq zero, zero, done

main:
lw a0, 0x4(zero) # size of array
lw a4, 0x8(zero) # address of array
addi a3, zero, 0
addi a2, zero, 2
beq a0, a3, done
lw a1, 0x0(a4)
for:
jal ra, prime
sw a1, 0x0(a4)
addi a3, a3, 1
addi a4, a4, 4
addi a2, zero, 2
beq a0, a3, done
lw a1, 0x0(a4)
beq zero, zero, for
