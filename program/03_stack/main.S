        .text
start:
        li a0, 128               # i = 1
counter:
        sb a0, 0(a0)             # push i
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push i
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push i
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push i
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        lb a0, 0(a0)             # pop t0
        lb a0, 0(a0)             # pop t0
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        addi a0, a0, 1           # i++
        sb a0, 0(a0)             # push t0
        lb a0, 0(a0)             # pop t0
        lb a0, 0(a0)             # pop t0
        lb a0, 0(a0)             # pop t0
        lb a0, 0(a0)             # pop t0
        lb a0, 0(a0)             # pop t0
        lb a0, 0(a0)             # pop t0
        lb a0, 0(a0)             # pop t0
        lb a0, 0(a0)             # pop t0
        beq zero, zero, counter  # if 0 == 0 then counter

