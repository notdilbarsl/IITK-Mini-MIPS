addi$1,$0,0;
addi$31,$0,5 // Replace 5 with size of your array
addi $1, $0 , 0;
addi $1 , $1 , 1;
beq $31 , $1 , 512;
addi $2, $1, -1
addi $5, $2, 1
lw $7, $2(0)
lw $8, $5(0)
bleq $8,$7,-7//code according to my conventions , basically bracnh if $8 >= $7
sw $7, $5(0)
sw $8, $2(0)
beq $2 , $0 , -10// (PC = 3)
addi $1, $1, 1
beq $0 , $0, -9// (PC = 6)