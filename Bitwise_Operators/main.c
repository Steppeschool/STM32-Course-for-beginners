#include <stdio.h>
#include <stdint.h>

void print_binary(uint8_t input)
{
 uint8_t i = (1 << (sizeof(input)*8-1));
 for(; i; i >>= 1)
 {
      printf("%d",(input&i)!=0);
 }
 printf("\n");
}

int main()
{
	uint8_t x,y,z;
	x = 0b11101001;
	y = 0b10010110;
	print_binary(x);
	print_binary(y);
	z = x & y; // AND, 10000000;
	printf("AND bitwise operator: \n");
	print_binary(z);

	z = x | y; // OR, 11111111;
	printf("OR bitwise operator: \n");
	print_binary(z);

	z = ~x ; // One's complement, 00010110;
	printf("One's complement bitwise operator: \n");
	print_binary(z);

	z = x ^ y; // XOR, 01111111        1^0 = 1, 0^1 =1
	printf("XOR bitwise operator: \n");
	print_binary(z);

	z = x >> 1; // Right shift, 01110100;
	printf("Right shift bitwise operator: \n");
	print_binary(z);

	z = x << 2; // LEFT shift, 10100100;
	printf("Left shift bitwise operator: \n");
	print_binary(z);
	printf("%d", z);
	return 0;
}









