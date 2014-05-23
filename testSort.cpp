#include <stdlib.h>
#include <stdio.h>

int a,b,c;


int compare(const void *a, const void *b)
{
	int *realA = *(int * const*) a;
	int *realB = *(int * const*) b;
	printf("%d %d\n",*realA,*realB);
	return (*realA < *realB) - (*realA > *realB);
}

int main()
{
	a = 1;
	b = 7;
	c = 5;
	
	printf("%d %d %d\n", &a, &b, &c);
	
	int *values[] = {&a,&b,&c};
	
	qsort(values, 3, sizeof(int*), compare);
	for (int i = 0; i < 3; i++)
	{
		int *val = values[i];
		printf("%d ", *val);
	}
	printf("\n");
}
